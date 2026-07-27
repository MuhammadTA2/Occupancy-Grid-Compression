#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include "pins.h"
#include "lora_transport.h"
#include "grid.h"
#include "rle.h"
#include "split_codec.h"
#include "packetizer.h"
#include "packet_reassembler.h"

using namespace compressor;

// Bidirectional protocol over one LoRa link, distinguished by messageId:
//   GRID_MESSAGE_ID             (rover -> base)  : occupancy grid, via splitcodec (values+counts streams)
//   FRAGMENT_STATUS_MESSAGE_ID  (base -> rover)  : which grid fragments base is still missing, so rover
//                                                  can resend exactly those instead of everything (ARQ)
//   INSTRUCTIONS_MESSAGE_ID     (base -> rover)  : movement waypoints, raw bytes (no RLE -- a handful of
//                                                  waypoints has nothing for run-length to exploit)
//
// ARQ over the grid link: base tracks a "quiet period" (QUIET_PERIOD_MS of no new grid fragments
// arriving) and, once elapsed without both streams complete, sends a status report naming which
// fragment indices are still missing per stream. Each stream's status is one of three states, not a
// plain missing-count, because "0 missing" is ambiguous between "confirmed complete" and "nothing
// received yet" (the reassembler can't report indices for a stream it doesn't know totalFragments
// for) -- conflating those would make the rover think an untouched stream had actually succeeded:
//   STATUS_COMPLETE          : this stream is fully received, no need to resend anything.
//   STATUS_PARTIAL           : some fragments arrived; report lists exactly which indices are missing.
//   STATUS_NOTHING_RECEIVED  : zero fragments arrived for this stream yet (totalFragments unknown to
//                              the reassembler) -- rover should resend everything, not specific indices.
// Rover retains its fragmented packet lists after the initial send (not just local variables) so it
// can look up and resend individual fragment indices later. It retries up to MAX_RETRY_ROUNDS times
// per stream; if no status report arrives at all within REPORT_TIMEOUT_MS, it treats the report
// itself as lost and falls back to resending everything for whichever stream isn't yet confirmed
// complete.
//
// Serial legs (real, not synthetic):
//   Jetson -> ROVER   : a flat vector of GRID_ROWS*GRID_COLS cell bytes (0/1/2), no header. Both ends
//                       must already agree on GRID_ROWS/GRID_COLS since this framing has no length
//                       field -- if the Jetson ever needs to send a *variable* size, this needs a
//                       [rows][cols] header added, matching the BASE->computer leg below.
//   BASE -> computer  : [rows:2][cols:2][cell bytes...], all little-endian.
//   computer -> BASE  : [waypointCount:1][x:2][y:2] repeated -- same (x,y) encoding as the LoRa leg,
//                       with a count prefix added since a raw serial stream needs an explicit boundary
//                       that the LoRa leg gets for free from the reassembled buffer's own length.
//   ROVER -> Jetson   : same [waypointCount:1][x:2][y:2] framing, forwarding what arrived over LoRa.
//                       Placeholder framing -- not yet confirmed against real Jetson-side code.
//
// Real path planning and real motor control are still separate, not-yet-built pieces on the
// computer and powertrain-ESP32 sides respectively; this firmware only owns the two ESP32s and the
// LoRa link between them.
namespace{
	const uint16_t GRID_MESSAGE_ID = 1;
	const uint16_t INSTRUCTIONS_MESSAGE_ID = 2;
	const uint16_t FRAGMENT_STATUS_MESSAGE_ID = 3;
	const uint8_t VALUES_STREAM_ID = 0;
	const uint8_t COUNTS_STREAM_ID = 1;
	const uint8_t INSTRUCTIONS_STREAM_ID = 0;
	const uint8_t STATUS_STREAM_ID = 0;
	const uint8_t VALUE_BIT_WIDTH = 2; // occupancy values are 0/1/2, 11 reserved

	// Fixed grid size shared by both ends of the Jetson<->ROVER serial link
	// (see header comment -- that framing has no length field of its own).
	const int GRID_ROWS = 20;
	const int GRID_COLS = 20;

	// ARQ tuning (see header comment for the overall design).
	const unsigned long QUIET_PERIOD_MS = 500;
	const unsigned long REPORT_TIMEOUT_MS = 2000;
	const int MAX_RETRY_ROUNDS = 5;

	const uint8_t STATUS_COMPLETE = 0;
	const uint8_t STATUS_PARTIAL = 1;
	const uint8_t STATUS_NOTHING_RECEIVED = 2;

	LoRaTransport transport;
	PacketReassembler reassembler;

	struct Waypoint{ int16_t x; int16_t y; };

	// No compression here on purpose -- unlike an occupancy grid, a short
	// waypoint list has no long runs for RLE to exploit, so this is just a
	// flat little-endian encoding.
	std::vector<uint8_t> encodeWaypoints(const std::vector<Waypoint>& waypoints){
		std::vector<uint8_t> bytes;
		bytes.reserve(waypoints.size() * 4);
		for(const Waypoint& w : waypoints){
			bytes.push_back(static_cast<uint8_t>(w.x & 0xFF));
			bytes.push_back(static_cast<uint8_t>((w.x >> 8) & 0xFF));
			bytes.push_back(static_cast<uint8_t>(w.y & 0xFF));
			bytes.push_back(static_cast<uint8_t>((w.y >> 8) & 0xFF));
		}
		return bytes;
	}

	std::vector<Waypoint> decodeWaypoints(const std::vector<uint8_t>& bytes){
		std::vector<Waypoint> waypoints;
		for(size_t i = 0; i + 4 <= bytes.size(); i += 4){
			int16_t x = static_cast<int16_t>(bytes[i] | (bytes[i + 1] << 8));
			int16_t y = static_cast<int16_t>(bytes[i + 2] | (bytes[i + 3] << 8));
			waypoints.push_back(Waypoint{x, y});
		}
		return waypoints;
	}

	// Fragment-status report, covering both grid streams in one message:
	//   [subjectMessageId:2]
	//   [valuesStatus:1] (+ [count:1][idx:2]*count only if valuesStatus == STATUS_PARTIAL)
	//   [countsStatus:1] (+ [count:1][idx:2]*count only if countsStatus == STATUS_PARTIAL)
	std::vector<uint8_t> encodeStatusReport(uint16_t subjectMessageId,
	                                         uint8_t valuesStatus, const std::vector<uint16_t>& valuesMissing,
	                                         uint8_t countsStatus, const std::vector<uint16_t>& countsMissing){
		std::vector<uint8_t> bytes;
		bytes.push_back(static_cast<uint8_t>(subjectMessageId & 0xFF));
		bytes.push_back(static_cast<uint8_t>((subjectMessageId >> 8) & 0xFF));

		bytes.push_back(valuesStatus);
		if(valuesStatus == STATUS_PARTIAL){
			bytes.push_back(static_cast<uint8_t>(valuesMissing.size()));
			for(uint16_t idx : valuesMissing){
				bytes.push_back(static_cast<uint8_t>(idx & 0xFF));
				bytes.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF));
			}
		}

		bytes.push_back(countsStatus);
		if(countsStatus == STATUS_PARTIAL){
			bytes.push_back(static_cast<uint8_t>(countsMissing.size()));
			for(uint16_t idx : countsMissing){
				bytes.push_back(static_cast<uint8_t>(idx & 0xFF));
				bytes.push_back(static_cast<uint8_t>((idx >> 8) & 0xFF));
			}
		}
		return bytes;
	}

	struct StatusReport{
		bool valid = false;
		uint16_t subjectMessageId = 0;
		uint8_t valuesStatus = STATUS_NOTHING_RECEIVED;
		std::vector<uint16_t> valuesMissing;
		uint8_t countsStatus = STATUS_NOTHING_RECEIVED;
		std::vector<uint16_t> countsMissing;
	};

	bool readMissingList(const std::vector<uint8_t>& bytes, size_t& pos, std::vector<uint16_t>& out){
		if(pos >= bytes.size()) return false;
		uint8_t count = bytes[pos++];
		for(uint8_t i = 0; i < count; i++){
			if(pos + 2 > bytes.size()) return false;
			uint16_t idx = static_cast<uint16_t>(bytes[pos]) | static_cast<uint16_t>(static_cast<uint16_t>(bytes[pos + 1]) << 8);
			out.push_back(idx);
			pos += 2;
		}
		return true;
	}

	StatusReport decodeStatusReport(const std::vector<uint8_t>& bytes){
		StatusReport report;
		if(bytes.size() < 4) return report; // truncated -- not even the two status bytes fit
		size_t pos = 0;
		report.subjectMessageId = static_cast<uint16_t>(bytes[0]) | static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8);
		pos = 2;

		report.valuesStatus = bytes[pos++];
		if(report.valuesStatus == STATUS_PARTIAL){
			if(!readMissingList(bytes, pos, report.valuesMissing)) return report;
		}

		if(pos >= bytes.size()) return report;
		report.countsStatus = bytes[pos++];
		if(report.countsStatus == STATUS_PARTIAL){
			if(!readMissingList(bytes, pos, report.countsMissing)) return report;
		}

		report.valid = true;
		return report;
	}

	void initLoRa(){
		Serial.begin(115200);
		delay(1000); // give the serial monitor time to attach

		// Serial2 (separate UART, separate pins -- see pins.h) carries only
		// the binary Jetson/computer protocol; Serial (USB) carries only
		// human-readable debug text. Keeping them apart means the
		// Jetson/computer side never has to distinguish debug output from
		// real payload bytes on the wire.
		Serial2.begin(DATA_UART_BAUD, SERIAL_8N1, DATA_UART_RX_PIN, DATA_UART_TX_PIN);

		LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
		if(!LoRa.begin(LORA_FREQUENCY_HZ)){
			Serial.println("LoRa.begin() failed -- check wiring and LORA_FREQUENCY_HZ (pins.h).");
			while(true) delay(1000);
		}
		Serial.println("LoRa radio initialized.");
	}

	std::vector<packetizer::Packet> sendStream(uint16_t messageId, const std::vector<uint8_t>& data, uint8_t streamId, const char* label){
		std::vector<packetizer::Packet> packets = packetizer::fragment(messageId, streamId, data);
		Serial.printf("Sending %s: %u bytes -> %u fragment(s)\n", label,
		              static_cast<unsigned>(data.size()), static_cast<unsigned>(packets.size()));

		for(const packetizer::Packet& p : packets){
			std::vector<uint8_t> serialized = packetizer::serialize(p);
			transport.send(serialized);
			Serial.printf("  sent fragment %u/%u (%u bytes on air)\n",
			              static_cast<unsigned>(p.header.fragmentIndex + 1),
			              static_cast<unsigned>(p.header.totalFragments),
			              static_cast<unsigned>(serialized.size()));
			delay(300); // leave airtime between packets rather than back-to-back
		}
		return packets;
	}

	void resendFragments(const std::vector<packetizer::Packet>& packets, const std::vector<uint16_t>& indices, const char* label){
		for(uint16_t idx : indices){
			if(idx >= packets.size()) continue; // defensive -- a well-formed report never triggers this
			transport.send(packetizer::serialize(packets[idx]));
			delay(300);
		}
		Serial.printf("Resent %u %s fragment(s)\n", static_cast<unsigned>(indices.size()), label);
	}

	void resendAllFragments(const std::vector<packetizer::Packet>& packets, const char* label){
		for(const packetizer::Packet& p : packets){
			transport.send(packetizer::serialize(p));
			delay(300);
		}
		Serial.printf("Resent all %u %s fragment(s)\n", static_cast<unsigned>(packets.size()), label);
	}

	// Blocks (yielding via delay(), not busy-spinning) until exactly `count`
	// bytes have arrived on Serial2 -- the dedicated binary data link, not
	// the USB debug console.
	std::vector<uint8_t> readExactDataBytes(size_t count){
		std::vector<uint8_t> buf;
		buf.reserve(count);
		while(buf.size() < count){
			if(Serial2.available()){
				buf.push_back(static_cast<uint8_t>(Serial2.read()));
			} else {
				delay(1);
			}
		}
		return buf;
	}
}

#if defined(DEVICE_ROLE_ROVER)

namespace{
	bool sentGrid = false;
	bool gridConfirmed = false; // both streams confirmed complete or given up on
	bool gotInstructions = false;

	std::vector<packetizer::Packet> gridValuesPackets;
	std::vector<packetizer::Packet> gridCountsPackets;
	bool valuesDone = false;
	bool countsDone = false;
	int valuesRetryRound = 0;
	int countsRetryRound = 0;
	unsigned long lastArqActivityMillis = 0;
}

void setup(){
	initLoRa();
	Serial.println("Role: ROVER");
}

void loop(){
	if(!sentGrid){
		Serial.printf("Waiting for a %dx%d grid (%d bytes) from the Jetson over Serial2...\n",
		              GRID_ROWS, GRID_COLS, GRID_ROWS * GRID_COLS);
		std::vector<uint8_t> cellBytes = readExactDataBytes(static_cast<size_t>(GRID_ROWS) * GRID_COLS);
		Serial.println("Grid received from Jetson.");

		Grid grid = rebuildGrid(cellBytes, GRID_ROWS, GRID_COLS);
		SymbolStream stream = toSymbolStream(grid);
		RLERuns runs = rleEncode(stream.symbols);
		splitcodec::EncodedStreams streams = splitcodec::encode(runs, VALUE_BIT_WIDTH);

		Serial.printf("Grid: %dx%d, %u cells, %u RLE runs\n", grid.rows, grid.cols,
		              static_cast<unsigned>(grid.data.size()), static_cast<unsigned>(runs.values.size()));

		gridValuesPackets = sendStream(GRID_MESSAGE_ID, streams.valuesBytes, VALUES_STREAM_ID, "grid values");
		gridCountsPackets = sendStream(GRID_MESSAGE_ID, streams.countsBytes, COUNTS_STREAM_ID, "grid counts");
		Serial.println("Grid sent. Waiting for fragment-status reports from base...");
		sentGrid = true;
		lastArqActivityMillis = millis();
		return;
	}

	if(!gridConfirmed){
		std::vector<uint8_t> raw;
		if(transport.receive(raw)){
			packetizer::Packet packet;
			if(packetizer::deserialize(raw, packet) && packet.header.messageId == FRAGMENT_STATUS_MESSAGE_ID){
				// Status reports are tiny (a handful of bytes) and always fit
				// in one fragment, so they're handled directly per-packet
				// rather than through the reassembler -- that sidesteps
				// having to detect "is this a new report or one I already
				// acted on" across repeated deliveries of the same messageId.
				StatusReport report = decodeStatusReport(packet.payload);
				if(report.valid && report.subjectMessageId == GRID_MESSAGE_ID){
					lastArqActivityMillis = millis();

					if(report.valuesStatus == STATUS_COMPLETE){
						valuesDone = true;
					} else if(!valuesDone && valuesRetryRound < MAX_RETRY_ROUNDS){
						valuesRetryRound++;
						if(report.valuesStatus == STATUS_PARTIAL) resendFragments(gridValuesPackets, report.valuesMissing, "values");
						else resendAllFragments(gridValuesPackets, "values"); // STATUS_NOTHING_RECEIVED
					} else if(!valuesDone){
						Serial.println("Giving up on values stream after max retries.");
						valuesDone = true;
					}

					if(report.countsStatus == STATUS_COMPLETE){
						countsDone = true;
					} else if(!countsDone && countsRetryRound < MAX_RETRY_ROUNDS){
						countsRetryRound++;
						if(report.countsStatus == STATUS_PARTIAL) resendFragments(gridCountsPackets, report.countsMissing, "counts");
						else resendAllFragments(gridCountsPackets, "counts"); // STATUS_NOTHING_RECEIVED
					} else if(!countsDone){
						Serial.println("Giving up on counts stream after max retries.");
						countsDone = true;
					}
				}
			}
		}

		if(!valuesDone || !countsDone){
			if(millis() - lastArqActivityMillis > REPORT_TIMEOUT_MS){
				Serial.println("No fragment-status report in time -- assuming it was lost, resending as a fallback.");
				if(!valuesDone){
					if(valuesRetryRound < MAX_RETRY_ROUNDS){ valuesRetryRound++; resendAllFragments(gridValuesPackets, "values"); }
					else { Serial.println("Giving up on values stream after max retries."); valuesDone = true; }
				}
				if(!countsDone){
					if(countsRetryRound < MAX_RETRY_ROUNDS){ countsRetryRound++; resendAllFragments(gridCountsPackets, "counts"); }
					else { Serial.println("Giving up on counts stream after max retries."); countsDone = true; }
				}
				lastArqActivityMillis = millis();
			}
			return;
		}

		Serial.println("Grid delivery confirmed (or given up on). Waiting for instructions from base...");
		gridConfirmed = true;
		return;
	}

	if(gotInstructions) return; // done for this first pass -- no re-scan loop yet

	std::vector<uint8_t> raw;
	if(!transport.receive(raw)) return;

	packetizer::Packet packet;
	if(!packetizer::deserialize(raw, packet)){
		Serial.println("Received a corrupted/malformed packet -- dropped.");
		return;
	}
	if(packet.header.messageId != INSTRUCTIONS_MESSAGE_ID) return; // not instructions traffic

	reassembler.receive(packet);
	Serial.printf("Got instructions fragment %u/%u, RSSI=%d\n",
	              static_cast<unsigned>(packet.header.fragmentIndex + 1),
	              static_cast<unsigned>(packet.header.totalFragments), LoRa.packetRssi());

	if(!reassembler.isComplete(INSTRUCTIONS_MESSAGE_ID, INSTRUCTIONS_STREAM_ID)) return;

	std::vector<uint8_t> instructionsBytes;
	reassembler.tryGetCompleteStream(INSTRUCTIONS_MESSAGE_ID, INSTRUCTIONS_STREAM_ID, instructionsBytes);
	std::vector<Waypoint> waypoints = decodeWaypoints(instructionsBytes);

	Serial.printf("Instructions received: %u waypoint(s)\n", static_cast<unsigned>(waypoints.size()));
	for(const Waypoint& w : waypoints){
		Serial.printf("  (%d, %d)\n", w.x, w.y);
	}

	// Forward to the Jetson over Serial2: [count:1][x:2][y:2] repeated.
	// Placeholder framing -- adjust if your Jetson-side code expects
	// something different; this wasn't specified yet.
	Serial2.write(static_cast<uint8_t>(waypoints.size()));
	std::vector<uint8_t> waypointBytes = encodeWaypoints(waypoints);
	Serial2.write(waypointBytes.data(), waypointBytes.size());

	gotInstructions = true;
}

#elif defined(DEVICE_ROLE_BASE)

namespace{
	bool gotGrid = false;
	bool sentInstructions = false;
	unsigned long lastGridFragmentMillis = 0;
	bool reportSentForThisQuietPeriod = false;
}

void setup(){
	initLoRa();
	Serial.println("Role: BASE -- waiting for grid...");
}

void loop(){
	if(sentInstructions) return; // done for this first pass

	std::vector<uint8_t> raw;
	if(transport.receive(raw)){
		packetizer::Packet packet;
		if(!packetizer::deserialize(raw, packet)){
			Serial.println("Received a corrupted/malformed packet -- dropped.");
		} else if(packet.header.messageId == GRID_MESSAGE_ID){
			reassembler.receive(packet);
			lastGridFragmentMillis = millis();
			reportSentForThisQuietPeriod = false;
			Serial.printf("Got grid fragment: streamId=%u %u/%u, RSSI=%d\n",
			              packet.header.streamId,
			              static_cast<unsigned>(packet.header.fragmentIndex + 1),
			              static_cast<unsigned>(packet.header.totalFragments), LoRa.packetRssi());
		}
		// other messageIds aren't expected in this phase -- ignored
	}

	if(gotGrid) return;

	bool valuesComplete = reassembler.isComplete(GRID_MESSAGE_ID, VALUES_STREAM_ID);
	bool countsComplete = reassembler.isComplete(GRID_MESSAGE_ID, COUNTS_STREAM_ID);

	if(valuesComplete && countsComplete){
		std::vector<uint8_t> valuesBytes, countsBytes;
		reassembler.tryGetCompleteStream(GRID_MESSAGE_ID, VALUES_STREAM_ID, valuesBytes);
		reassembler.tryGetCompleteStream(GRID_MESSAGE_ID, COUNTS_STREAM_ID, countsBytes);

		RLERuns runs;
		bool decodeOk = splitcodec::decode(valuesBytes, countsBytes, VALUE_BIT_WIDTH, runs);
		if(!decodeOk){
			Serial.println("splitcodec::decode failed -- both streams complete but data didn't decode.");
			gotGrid = true;
			return;
		}

		std::vector<uint8_t> symbols = rleDecode(runs);
		Serial.printf("Grid decoded: %u cells, %u RLE runs\n",
		              static_cast<unsigned>(symbols.size()), static_cast<unsigned>(runs.values.size()));

		// Final explicit "you're done" status report -- without this, the
		// rover has no unambiguous signal that both streams succeeded (it
		// would just eventually time out on the last resend and give up).
		std::vector<uint8_t> finalReport = encodeStatusReport(GRID_MESSAGE_ID, STATUS_COMPLETE, {}, STATUS_COMPLETE, {});
		sendStream(FRAGMENT_STATUS_MESSAGE_ID, finalReport, STATUS_STREAM_ID, "final fragment status (complete)");
		gotGrid = true;

		// Forward the decompressed grid to the base-station computer over
		// Serial2: [rows:2][cols:2][cell bytes...], little-endian. No
		// trailing terminator -- the computer-side reader must read exactly
		// 4 + rows*cols bytes and stop there.
		Serial2.write(static_cast<uint8_t>(GRID_ROWS & 0xFF));
		Serial2.write(static_cast<uint8_t>((GRID_ROWS >> 8) & 0xFF));
		Serial2.write(static_cast<uint8_t>(GRID_COLS & 0xFF));
		Serial2.write(static_cast<uint8_t>((GRID_COLS >> 8) & 0xFF));
		Serial2.write(symbols.data(), symbols.size());
		Serial.println("Grid forwarded to base-station computer over Serial2. Waiting for path instructions...");

		// Block until the base-station computer sends back:
		// [waypointCount:1][x:2][y:2] repeated.
		std::vector<uint8_t> countByte = readExactDataBytes(1);
		uint8_t waypointCount = countByte[0];
		std::vector<uint8_t> waypointBytes = readExactDataBytes(static_cast<size_t>(waypointCount) * 4);
		std::vector<Waypoint> path = decodeWaypoints(waypointBytes);
		Serial.printf("Received %u waypoint(s) from the base-station computer.\n", static_cast<unsigned>(path.size()));

		std::vector<uint8_t> instructionsBytes = encodeWaypoints(path);
		Serial.println("Sending instructions back to rover...");
		sendStream(INSTRUCTIONS_MESSAGE_ID, instructionsBytes, INSTRUCTIONS_STREAM_ID, "instructions");
		sentInstructions = true;
		return;
	}

	// Not complete yet -- report what's missing, but only once per quiet
	// period (reset above whenever a new fragment actually arrives), and
	// only once we've heard from the rover at all.
	if(!reportSentForThisQuietPeriod && lastGridFragmentMillis != 0 &&
	   millis() - lastGridFragmentMillis > QUIET_PERIOD_MS){
		uint8_t valuesStatus;
		std::vector<uint16_t> valuesMissing;
		if(valuesComplete){
			valuesStatus = STATUS_COMPLETE;
		} else {
			valuesMissing = reassembler.missingFragments(GRID_MESSAGE_ID, VALUES_STREAM_ID);
			// An empty list while not complete means the reassembler never
			// learned totalFragments for this stream -- i.e. nothing arrived
			// yet -- not that nothing is missing.
			valuesStatus = valuesMissing.empty() ? STATUS_NOTHING_RECEIVED : STATUS_PARTIAL;
		}

		uint8_t countsStatus;
		std::vector<uint16_t> countsMissing;
		if(countsComplete){
			countsStatus = STATUS_COMPLETE;
		} else {
			countsMissing = reassembler.missingFragments(GRID_MESSAGE_ID, COUNTS_STREAM_ID);
			countsStatus = countsMissing.empty() ? STATUS_NOTHING_RECEIVED : STATUS_PARTIAL;
		}

		Serial.println("Quiet period elapsed with the grid still incomplete -- sending fragment-status report.");
		std::vector<uint8_t> reportBytes = encodeStatusReport(GRID_MESSAGE_ID, valuesStatus, valuesMissing, countsStatus, countsMissing);
		sendStream(FRAGMENT_STATUS_MESSAGE_ID, reportBytes, STATUS_STREAM_ID, "fragment status");
		reportSentForThisQuietPeriod = true;
	}
}

#else
	#error "Build with -D DEVICE_ROLE_ROVER or -D DEVICE_ROLE_BASE (see platformio.ini environments)"
#endif
