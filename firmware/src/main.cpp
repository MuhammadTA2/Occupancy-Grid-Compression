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

// Bidirectional protocol over one LoRa link, distinguished by messageId, run
// in a continuous loop of rounds (one round = one grid down, one
// instructions set back) until the device is reset or power-cycled:
//   grid messages          (rover -> base)  : occupancy grid, via splitcodec (values+counts streams),
//                                              plus the rover's own grid-cell position (position
//                                              stream) so base can place the local scan on its master
//                                              map. messageId = GRID_MESSAGE_ID_BASE + roundNumber.
//                                              The grid's [rows:2][cols:2] rides as a 4-byte prefix on
//                                              the values stream's payload (not a separate tracked
//                                              stream) so it gets the same fragmentation/ARQ
//                                              guarantees for free. It's redundant with the fixed
//                                              GRID_ROWS/GRID_COLS below, and deliberately so: it's
//                                              the only end-to-end check that what base decoded has
//                                              the shape rover actually sent.
//   FRAGMENT_STATUS_MESSAGE_ID  (base -> rover)  : which grid fragments base is still missing, so rover
//                                                  can resend exactly those instead of everything (ARQ).
//                                                  Always sent with this fixed messageId (it's a type
//                                                  tag, not round-scoped) -- the round it's *about* is
//                                                  carried inside the payload as subjectMessageId.
//   instructions messages  (base -> rover)  : movement waypoints, raw bytes (no RLE -- a handful of
//                                              waypoints has nothing for run-length to exploit).
//                                              messageId = INSTRUCTIONS_MESSAGE_ID_BASE + roundNumber.
//
// Round-scoped message IDs (rather than one fixed ID reused every round) exist because
// PacketReassembler has no reset: it's keyed by (messageId, streamId), so isComplete() would
// instantly return true on round 2 from round 1's leftover data before any new fragment arrived, if
// the ID didn't change. Giving each round a fresh ID sidesteps that entirely -- old rounds' entries
// just sit unused in the reassembler's map, which is fine for a bounded test session.
//
// ARQ over the grid link: base tracks a "quiet period" (QUIET_PERIOD_MS of no new grid fragments
// arriving) and, once elapsed without all three streams complete, sends a status report naming which
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
// The instructions leg is acknowledged explicitly: once rover finishes reassembling a complete
// instructions message, it sends INSTRUCTIONS_ACK_MESSAGE_ID back naming which messageId it got. Base
// waits up to INSTRUCTIONS_ACK_TIMEOUT_MS for that ack after sending, and resends (up to
// INSTRUCTIONS_MAX_RETRIES times) if it doesn't arrive. Separately, rover gives up waiting entirely
// after INSTRUCTIONS_TIMEOUT_MS from when it sent the grid, reporting 0 waypoints for that round to
// the PC and moving on to the next round rather than blocking forever -- that's the backstop for when
// every retry (and the ack for whichever one landed) is lost, not just the first attempt.
//
// Serial legs (real, not synthetic). Both USB legs are fixed-size and header-less: there is no
// length field and no framing byte anywhere on them, so both ends must already agree on
// GRID_ROWS/GRID_COLS, and a single dropped byte desyncs the leg permanently. That shape is set by
// the real producers/consumers, not chosen here -- the Jetson's writeGridAndPose() (src/esp_comm.cpp)
// and the base station's main_base.py/map_dis.py (LOCAL_SIZE) both assume exactly this:
//   Jetson -> ROVER   : [cell bytes...][x:2][y:2], little-endian, no header. Exactly
//                       GRID_ROWS*GRID_COLS cells, then the rover's grid-cell position.
//   BASE -> computer  : [cell bytes...][x:2][y:2], little-endian, no header -- same fixed-size shape
//                       as the Jetson->ROVER leg. Base drops the round rather than writing anything
//                       if what it decoded isn't exactly GRID_ROWS x GRID_COLS, so a shape mismatch
//                       fails visibly instead of silently shifting every later round on this leg.
//   computer -> BASE  : [waypointCount:1][x:2][y:2] repeated -- same (x,y) encoding as the LoRa leg,
//                       with a count prefix added since a raw serial stream needs an explicit boundary
//                       that the LoRa leg gets for free from the reassembled buffer's own length.
//   ROVER -> Jetson   : same [waypointCount:1][x:2][y:2] framing, forwarding what arrived over LoRa.
//                       waypointCount == 0 specifically means "base never confirmed this round" (either
//                       the grid ARQ gave up, or the instructions leg timed out), not "zero waypoints
//                       computed". Placeholder framing -- not yet confirmed against real Jetson-side code.
//
// Real path planning and real motor control are still separate, not-yet-built pieces on the
// computer and powertrain-ESP32 sides respectively; this firmware only owns the two ESP32s and the
// LoRa link between them.
//
// This bench-test build has no external USB-to-serial adapter wired to
// Serial2/GPIO16-17 -- both ESP32s reach the PC only through their own
// programming-port USB (`Serial`). So the binary Jetson/computer protocol
// (the "Serial legs" above) travels over Serial instead of Serial2, and
// human-readable debug prints are compiled out by default (mixing them with
// framed binary data would make the PC-side reader misparse debug text as
// payload bytes). Build with `-D ENABLE_SERIAL_DEBUG` to get prints back for
// standalone LoRa-only testing where nothing is reading Serial as a data
// channel.
#if defined(ENABLE_SERIAL_DEBUG)
	#define DBG_PRINTLN(x) Serial.println(x)
	#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
	#define DBG_PRINTLN(x)
	#define DBG_PRINTF(...)
#endif

namespace{
	const uint16_t GRID_MESSAGE_ID_BASE = 1000;
	const uint16_t INSTRUCTIONS_MESSAGE_ID_BASE = 2000;
	const uint16_t FRAGMENT_STATUS_MESSAGE_ID = 3;
	const uint16_t INSTRUCTIONS_ACK_MESSAGE_ID = 4;
	const uint8_t VALUES_STREAM_ID = 0;
	const uint8_t COUNTS_STREAM_ID = 1;
	const uint8_t POSITION_STREAM_ID = 2;
	const uint8_t INSTRUCTIONS_STREAM_ID = 0;
	const uint8_t STATUS_STREAM_ID = 0;
	const uint8_t INSTRUCTIONS_ACK_STREAM_ID = 0;
	const uint8_t VALUE_BIT_WIDTH = 2; // occupancy values are 0/1/2, 11 reserved
	const size_t POSITION_BYTES = 4; // [x:2][y:2], little-endian int16

	// Fixed grid size shared by both ends of both USB legs -- see the header
	// comment: that framing carries no length field, so this constant is the
	// only thing making it parseable. Changing it here means changing it in
	// mapping.cpp, base/main_base.py (via map_dis.LOCAL_SIZE) and both
	// test_scripts/ simulators too; there is no shared source of truth yet.
	const int GRID_ROWS = 70;
	const int GRID_COLS = 70;

	uint16_t gridMessageIdForRound(uint16_t round){ return static_cast<uint16_t>(GRID_MESSAGE_ID_BASE + round); }
	uint16_t instructionsMessageIdForRound(uint16_t round){ return static_cast<uint16_t>(INSTRUCTIONS_MESSAGE_ID_BASE + round); }
	bool isGridMessageId(uint16_t id){ return id >= GRID_MESSAGE_ID_BASE && id < INSTRUCTIONS_MESSAGE_ID_BASE; }
	uint16_t roundOfGridMessageId(uint16_t id){ return static_cast<uint16_t>(id - GRID_MESSAGE_ID_BASE); }

	// ARQ tuning (see header comment for the overall design).
	const unsigned long QUIET_PERIOD_MS = 500;
	const unsigned long REPORT_TIMEOUT_MS = 2000;
	const int MAX_RETRY_ROUNDS = 5;
	const unsigned long INSTRUCTIONS_ACK_TIMEOUT_MS = 3000;
	const int INSTRUCTIONS_MAX_RETRIES = 4;
	// Rover's overall give-up budget for a round, counted from grid-send. The
	// naive sum of the other timeouts here ((MAX_RETRY_ROUNDS * REPORT_TIMEOUT_MS)
	// + ((INSTRUCTIONS_MAX_RETRIES+1) * INSTRUCTIONS_ACK_TIMEOUT_MS) ~= 25s)
	// deliberately does NOT cover this -- it only counts the artificial delay(300)
	// pacing between packets, not actual LoRa airtime per transmission, which adds
	// on top of every send/resend in both phases and was underestimated in an
	// earlier pass (confirmed on hardware: real rounds were regularly taking
	// close to 2x that budget). This has a wide margin over the naive sum for
	// that reason, not because the naive sum was recomputed wrong.
	const unsigned long INSTRUCTIONS_TIMEOUT_MS = 60000;

	const uint8_t STATUS_COMPLETE = 0;
	const uint8_t STATUS_PARTIAL = 1;
	const uint8_t STATUS_NOTHING_RECEIVED = 2;

	LoRaTransport transport;
	PacketReassembler reassembler;

	// INSTRUCTIONS_ACK_MESSAGE_ID payload: [subjectMessageId:2] -- which
	// instructions messageId this ack is for.
	std::vector<uint8_t> encodeInstructionsAck(uint16_t subjectMessageId){
		return { static_cast<uint8_t>(subjectMessageId & 0xFF), static_cast<uint8_t>((subjectMessageId >> 8) & 0xFF) };
	}
	bool decodeInstructionsAck(const std::vector<uint8_t>& bytes, uint16_t& subjectMessageId){
		if(bytes.size() < 2) return false;
		subjectMessageId = static_cast<uint16_t>(bytes[0]) | static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8);
		return true;
	}

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

	// Fragment-status report, covering all three grid streams in one message:
	//   [subjectMessageId:2]
	//   [valuesStatus:1]   (+ [count:1][idx:2]*count only if valuesStatus == STATUS_PARTIAL)
	//   [countsStatus:1]   (+ [count:1][idx:2]*count only if countsStatus == STATUS_PARTIAL)
	//   [positionStatus:1] (+ [count:1][idx:2]*count only if positionStatus == STATUS_PARTIAL)
	std::vector<uint8_t> encodeStatusReport(uint16_t subjectMessageId,
	                                         uint8_t valuesStatus, const std::vector<uint16_t>& valuesMissing,
	                                         uint8_t countsStatus, const std::vector<uint16_t>& countsMissing,
	                                         uint8_t positionStatus, const std::vector<uint16_t>& positionMissing){
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

		bytes.push_back(positionStatus);
		if(positionStatus == STATUS_PARTIAL){
			bytes.push_back(static_cast<uint8_t>(positionMissing.size()));
			for(uint16_t idx : positionMissing){
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
		uint8_t positionStatus = STATUS_NOTHING_RECEIVED;
		std::vector<uint16_t> positionMissing;
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
		if(bytes.size() < 5) return report; // truncated -- not even the three status bytes fit
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

		if(pos >= bytes.size()) return report;
		report.positionStatus = bytes[pos++];
		if(report.positionStatus == STATUS_PARTIAL){
			if(!readMissingList(bytes, pos, report.positionMissing)) return report;
		}

		report.valid = true;
		return report;
	}

	void initLoRa(){
		Serial.begin(115200);
		delay(1000); // give the PC-side script/monitor time to attach after the USB-open auto-reset

		LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
		if(!LoRa.begin(LORA_FREQUENCY_HZ)){
			DBG_PRINTLN("LoRa.begin() failed -- check wiring and LORA_FREQUENCY_HZ (pins.h).");
			while(true) delay(1000);
		}
		// Without this, radio-level noise near the sensitivity floor can be
		// misreported by the SX127x as a genuinely "received" packet -- the
		// packetizer's own CRC-16 (packetizer::deserialize) catches most of
		// that downstream, but there's no reason to skip the radio's own
		// hardware CRC check when it's this cheap to enable.
		LoRa.enableCrc();
		DBG_PRINTLN("LoRa radio initialized.");
	}

	std::vector<packetizer::Packet> sendStream(uint16_t messageId, const std::vector<uint8_t>& data, uint8_t streamId, const char* label){
		std::vector<packetizer::Packet> packets = packetizer::fragment(messageId, streamId, data);
		DBG_PRINTF("Sending %s: %u bytes -> %u fragment(s)\n", label,
		           static_cast<unsigned>(data.size()), static_cast<unsigned>(packets.size()));

		for(const packetizer::Packet& p : packets){
			std::vector<uint8_t> serialized = packetizer::serialize(p);
			transport.send(serialized);
			DBG_PRINTF("  sent fragment %u/%u (%u bytes on air)\n",
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
		DBG_PRINTF("Resent %u %s fragment(s)\n", static_cast<unsigned>(indices.size()), label);
	}

	void resendAllFragments(const std::vector<packetizer::Packet>& packets, const char* label){
		for(const packetizer::Packet& p : packets){
			transport.send(packetizer::serialize(p));
			delay(300);
		}
		DBG_PRINTF("Resent all %u %s fragment(s)\n", static_cast<unsigned>(packets.size()), label);
	}

	// Blocks (yielding via delay(), not busy-spinning) until exactly `count`
	// bytes have arrived on Serial -- the native-USB link doubling as the
	// binary data channel in this bench-test build (see header comment).
	std::vector<uint8_t> readExactDataBytes(size_t count){
		std::vector<uint8_t> buf;
		buf.reserve(count);
		while(buf.size() < count){
			if(Serial.available()){
				buf.push_back(static_cast<uint8_t>(Serial.read()));
			} else {
				delay(1);
			}
		}
		return buf;
	}
}

#if defined(DEVICE_ROLE_ROVER)

namespace{
	uint16_t roundNumber = 0;
	bool sentGrid = false;
	bool gridConfirmed = false; // all three streams confirmed complete or given up on
	bool gotInstructions = false;

	uint16_t currentGridMsgId = 0;
	uint16_t currentInstructionsMsgId = 0;
	unsigned long instructionsWaitStartMillis = 0;

	std::vector<packetizer::Packet> gridValuesPackets;
	std::vector<packetizer::Packet> gridCountsPackets;
	std::vector<packetizer::Packet> gridPositionPackets;
	bool valuesDone = false;
	bool countsDone = false;
	bool positionDone = false;
	int valuesRetryRound = 0;
	int countsRetryRound = 0;
	int positionRetryRound = 0;
	unsigned long lastArqActivityMillis = 0;
}

void setup(){
	initLoRa();
	DBG_PRINTLN("Role: ROVER");
}

void loop(){
	// Receive and route at most one incoming packet per iteration, regardless of which
	// stage we're nominally in below. Base doesn't wait for us to be "ready" before
	// sending -- it fires the instructions message the moment it has a PC response, which
	// can easily land while we're still deep in the grid-ARQ retry loop (that can take up
	// to ~10s if a status report gets lost). If routing were gated by stage, an early
	// instructions packet would just be silently dropped and never seen again -- which is
	// exactly what was happening. So routing here only depends on message type/id, not on
	// which stage flags happen to be set.
	if(sentGrid && !gotInstructions){
		std::vector<uint8_t> raw;
		if(transport.receive(raw)){
			packetizer::Packet packet;
			if(!packetizer::deserialize(raw, packet)){
				DBG_PRINTLN("Received a corrupted/malformed packet -- dropped.");
			} else if(!gridConfirmed && packet.header.messageId == FRAGMENT_STATUS_MESSAGE_ID){
				// Status reports are tiny (a handful of bytes) and always fit
				// in one fragment, so they're handled directly per-packet
				// rather than through the reassembler -- that sidesteps
				// having to detect "is this a new report or one I already
				// acted on" across repeated deliveries of the same messageId.
				StatusReport report = decodeStatusReport(packet.payload);
				if(report.valid && report.subjectMessageId == currentGridMsgId){
					lastArqActivityMillis = millis();

					if(report.valuesStatus == STATUS_COMPLETE){
						valuesDone = true;
					} else if(!valuesDone && valuesRetryRound < MAX_RETRY_ROUNDS){
						valuesRetryRound++;
						if(report.valuesStatus == STATUS_PARTIAL) resendFragments(gridValuesPackets, report.valuesMissing, "values");
						else resendAllFragments(gridValuesPackets, "values"); // STATUS_NOTHING_RECEIVED
					} else if(!valuesDone){
						DBG_PRINTLN("Giving up on values stream after max retries.");
						valuesDone = true;
					}

					if(report.countsStatus == STATUS_COMPLETE){
						countsDone = true;
					} else if(!countsDone && countsRetryRound < MAX_RETRY_ROUNDS){
						countsRetryRound++;
						if(report.countsStatus == STATUS_PARTIAL) resendFragments(gridCountsPackets, report.countsMissing, "counts");
						else resendAllFragments(gridCountsPackets, "counts"); // STATUS_NOTHING_RECEIVED
					} else if(!countsDone){
						DBG_PRINTLN("Giving up on counts stream after max retries.");
						countsDone = true;
					}

					if(report.positionStatus == STATUS_COMPLETE){
						positionDone = true;
					} else if(!positionDone && positionRetryRound < MAX_RETRY_ROUNDS){
						positionRetryRound++;
						if(report.positionStatus == STATUS_PARTIAL) resendFragments(gridPositionPackets, report.positionMissing, "position");
						else resendAllFragments(gridPositionPackets, "position"); // STATUS_NOTHING_RECEIVED
					} else if(!positionDone){
						DBG_PRINTLN("Giving up on position stream after max retries.");
						positionDone = true;
					}
				}
			} else if(packet.header.messageId == currentInstructionsMsgId){
				reassembler.receive(packet);
				DBG_PRINTF("Got instructions fragment %u/%u, RSSI=%d\n",
				           static_cast<unsigned>(packet.header.fragmentIndex + 1),
				           static_cast<unsigned>(packet.header.totalFragments), LoRa.packetRssi());

				if(reassembler.isComplete(currentInstructionsMsgId, INSTRUCTIONS_STREAM_ID)){
					std::vector<uint8_t> instructionsBytes;
					reassembler.tryGetCompleteStream(currentInstructionsMsgId, INSTRUCTIONS_STREAM_ID, instructionsBytes);

					// Sanity check before trusting this as real waypoint data. Each
					// waypoint is exactly 4 bytes and a real path here is never more
					// than a handful of points -- a well-formed, CRC-passing packet
					// landing outside that range points to something wrong upstream
					// of this decode (unconfirmed: possibly LoRa-level cross-talk
					// given how close the two antennas are on this bench setup, not
					// yet root-caused), not real data. Treat it as not-yet-complete
					// rather than forwarding garbage to the PC -- don't ack it, so
					// base's retry gets another chance to land cleanly, and the
					// existing per-round give-up timeout still bounds how long this
					// can stall a round.
					const size_t MAX_WAYPOINTS = 64;
					if(instructionsBytes.size() % 4 != 0 || instructionsBytes.size() / 4 > MAX_WAYPOINTS){
						DBG_PRINTF("Rejected implausible instructions payload (%u bytes) -- still waiting.\n",
						           static_cast<unsigned>(instructionsBytes.size()));
						return;
					}

					std::vector<Waypoint> waypoints = decodeWaypoints(instructionsBytes);

					DBG_PRINTF("Instructions received: %u waypoint(s)\n", static_cast<unsigned>(waypoints.size()));
					for(const Waypoint& w : waypoints){
						DBG_PRINTF("  (%d, %d)\n", w.x, w.y);
					}

					// Forward to the PC over Serial: [count:1][x:2][y:2] repeated.
					// Placeholder framing -- adjust if your real Jetson-side code
					// expects something different; this wasn't specified yet.
					Serial.write(static_cast<uint8_t>(waypoints.size()));
					std::vector<uint8_t> waypointBytes = encodeWaypoints(waypoints);
					Serial.write(waypointBytes.data(), waypointBytes.size());

					// Ack so base can stop retrying this round's instructions. If
					// the ack itself is lost, base will resend the whole message;
					// we won't be listening for it anymore (see the gate above)
					// since we've already gotten and forwarded it, so it'll just
					// go unacknowledged and base gives up after its own retries --
					// harmless, just wasted airtime, not a stuck round.
					sendStream(INSTRUCTIONS_ACK_MESSAGE_ID, encodeInstructionsAck(currentInstructionsMsgId), INSTRUCTIONS_ACK_STREAM_ID, "instructions ack");

					// Instructions in hand makes grid-ARQ moot even if it hadn't
					// formally finished yet -- settle it so the round-complete
					// step below runs cleanly.
					gotInstructions = true;
					gridConfirmed = true;
					valuesDone = true;
					countsDone = true;
					positionDone = true;
				}
			}
			// anything else (e.g. a stale report/instructions fragment from an
			// older round) -- ignored
		}
	}

	if(!sentGrid){
		roundNumber++;
		currentGridMsgId = gridMessageIdForRound(roundNumber);
		currentInstructionsMsgId = instructionsMessageIdForRound(roundNumber);

		// Fixed-size, header-less read -- see the header comment. There's no
		// length field to validate, so there's nothing to reject here either:
		// if this leg has desynced, it desynced silently and stays that way.
		const int gridRows = GRID_ROWS;
		const int gridCols = GRID_COLS;
		DBG_PRINTF("Round %u: reading %dx%d cells + position from the PC over Serial...\n",
		           static_cast<unsigned>(roundNumber), gridRows, gridCols);
		std::vector<uint8_t> cellBytes = readExactDataBytes(static_cast<size_t>(gridRows) * gridCols);
		std::vector<uint8_t> positionBytes = readExactDataBytes(POSITION_BYTES);
		DBG_PRINTF("Grid + position received from PC (position %d, %d).\n",
		           static_cast<int>(static_cast<int16_t>(positionBytes[0] | (positionBytes[1] << 8))),
		           static_cast<int>(static_cast<int16_t>(positionBytes[2] | (positionBytes[3] << 8))));

		Grid grid = rebuildGrid(cellBytes, gridRows, gridCols);
		SymbolStream stream = toSymbolStream(grid);
		RLERuns runs = rleEncode(stream.symbols);
		splitcodec::EncodedStreams streams = splitcodec::encode(runs, VALUE_BIT_WIDTH);

		DBG_PRINTF("Grid: %dx%d, %u cells, %u RLE runs\n", grid.rows, grid.cols,
		           static_cast<unsigned>(grid.data.size()), static_cast<unsigned>(runs.values.size()));

		// The grid's [rows:2][cols:2] rides as a 4-byte prefix on the values
		// stream's payload -- base needs to know the shape too (not just the
		// cell count), and prefixing it here reuses the values stream's
		// existing fragmentation/ARQ guarantees instead of needing a separate
		// tracked stream just for 4 bytes.
		std::vector<uint8_t> valuesPayload;
		valuesPayload.reserve(4 + streams.valuesBytes.size());
		valuesPayload.push_back(static_cast<uint8_t>(gridRows & 0xFF));
		valuesPayload.push_back(static_cast<uint8_t>((gridRows >> 8) & 0xFF));
		valuesPayload.push_back(static_cast<uint8_t>(gridCols & 0xFF));
		valuesPayload.push_back(static_cast<uint8_t>((gridCols >> 8) & 0xFF));
		valuesPayload.insert(valuesPayload.end(), streams.valuesBytes.begin(), streams.valuesBytes.end());

		gridValuesPackets = sendStream(currentGridMsgId, valuesPayload, VALUES_STREAM_ID, "grid values");
		gridCountsPackets = sendStream(currentGridMsgId, streams.countsBytes, COUNTS_STREAM_ID, "grid counts");
		gridPositionPackets = sendStream(currentGridMsgId, positionBytes, POSITION_STREAM_ID, "position");
		DBG_PRINTLN("Grid + position sent. Waiting for fragment-status reports from base...");
		sentGrid = true;
		lastArqActivityMillis = millis();
		// Counted from grid-send, not from grid confirmation -- instructions can
		// legitimately arrive (and be accepted, per the routing above) before we've
		// even finished confirming the grid, so the round's overall deadline has to
		// start from the same point regardless of how long confirmation takes.
		instructionsWaitStartMillis = millis();
		return;
	}

	if(gotInstructions){
		// Round complete (success or given up on) -- reset and go around again for the next grid.
		DBG_PRINTF("Round %u complete. Waiting for the next grid...\n", static_cast<unsigned>(roundNumber));
		sentGrid = false;
		gridConfirmed = false;
		gotInstructions = false;
		valuesDone = false;
		countsDone = false;
		positionDone = false;
		valuesRetryRound = 0;
		countsRetryRound = 0;
		positionRetryRound = 0;
		gridValuesPackets.clear();
		gridCountsPackets.clear();
		gridPositionPackets.clear();
		return;
	}

	if(!gridConfirmed){
		if(!valuesDone || !countsDone || !positionDone){
			if(millis() - lastArqActivityMillis > REPORT_TIMEOUT_MS){
				DBG_PRINTLN("No fragment-status report in time -- assuming it was lost, resending as a fallback.");
				if(!valuesDone){
					if(valuesRetryRound < MAX_RETRY_ROUNDS){ valuesRetryRound++; resendAllFragments(gridValuesPackets, "values"); }
					else { DBG_PRINTLN("Giving up on values stream after max retries."); valuesDone = true; }
				}
				if(!countsDone){
					if(countsRetryRound < MAX_RETRY_ROUNDS){ countsRetryRound++; resendAllFragments(gridCountsPackets, "counts"); }
					else { DBG_PRINTLN("Giving up on counts stream after max retries."); countsDone = true; }
				}
				if(!positionDone){
					if(positionRetryRound < MAX_RETRY_ROUNDS){ positionRetryRound++; resendAllFragments(gridPositionPackets, "position"); }
					else { DBG_PRINTLN("Giving up on position stream after max retries."); positionDone = true; }
				}
				lastArqActivityMillis = millis();
			}
		}
		if(valuesDone && countsDone && positionDone){
			DBG_PRINTLN("Grid delivery confirmed (or given up on). Waiting for instructions from base...");
			gridConfirmed = true;
		}
	}

	// Whether or not grid confirmation finished, give up on instructions after the
	// same fixed deadline from grid-send -- a stuck ARQ stage shouldn't get the round
	// an unbounded extra allowance to also wait on instructions afterwards.
	if(millis() - instructionsWaitStartMillis > INSTRUCTIONS_TIMEOUT_MS){
		DBG_PRINTLN("No instructions from base in time -- giving up on this round.");
		Serial.write(static_cast<uint8_t>(0)); // tell the PC: no waypoints this round
		gotInstructions = true; // picked up by the round-complete branch above, next loop() call
	}
}

#elif defined(DEVICE_ROLE_BASE)

namespace{
	uint16_t lastSeenGridRound = 0; // 0 = no round seen yet
	bool gotGrid = false;
	unsigned long lastGridFragmentMillis = 0;
	bool reportSentForThisQuietPeriod = false;
}

void setup(){
	initLoRa();
	DBG_PRINTLN("Role: BASE -- waiting for grid...");
}

void loop(){
	std::vector<uint8_t> raw;
	if(transport.receive(raw)){
		packetizer::Packet packet;
		if(!packetizer::deserialize(raw, packet)){
			DBG_PRINTLN("Received a corrupted/malformed packet -- dropped.");
		} else if(isGridMessageId(packet.header.messageId)){
			uint16_t round = roundOfGridMessageId(packet.header.messageId);
			if(round > lastSeenGridRound){
				// A genuinely new round has started -- forget the previous round's
				// completion state. The reassembler itself doesn't need clearing:
				// this round's messageId is unique, so isComplete()/etc. start
				// fresh for that key.
				lastSeenGridRound = round;
				gotGrid = false;
				reportSentForThisQuietPeriod = false;
			}
			if(round == lastSeenGridRound){
				reassembler.receive(packet);
				lastGridFragmentMillis = millis();
				reportSentForThisQuietPeriod = false;
				DBG_PRINTF("Round %u: got grid fragment streamId=%u %u/%u, RSSI=%d\n",
				           static_cast<unsigned>(round), packet.header.streamId,
				           static_cast<unsigned>(packet.header.fragmentIndex + 1),
				           static_cast<unsigned>(packet.header.totalFragments), LoRa.packetRssi());
			}
			// round < lastSeenGridRound: a straggler from an already-finished
			// older round (e.g. a late ARQ retransmission) -- ignored outright,
			// specifically without touching lastSeenGridRound/gotGrid, so it
			// can't snap the round tracking backward mid-decode.
		}
		// other messageIds (e.g. stale/duplicate status reports) aren't expected here -- ignored
	}

	if(gotGrid || lastSeenGridRound == 0) return;

	uint16_t currentGridMsgId = gridMessageIdForRound(lastSeenGridRound);
	bool valuesComplete = reassembler.isComplete(currentGridMsgId, VALUES_STREAM_ID);
	bool countsComplete = reassembler.isComplete(currentGridMsgId, COUNTS_STREAM_ID);
	bool positionComplete = reassembler.isComplete(currentGridMsgId, POSITION_STREAM_ID);

	if(valuesComplete && countsComplete && positionComplete){
		std::vector<uint8_t> valuesBytesWithHeader, countsBytes, positionBytes;
		reassembler.tryGetCompleteStream(currentGridMsgId, VALUES_STREAM_ID, valuesBytesWithHeader);
		reassembler.tryGetCompleteStream(currentGridMsgId, COUNTS_STREAM_ID, countsBytes);
		reassembler.tryGetCompleteStream(currentGridMsgId, POSITION_STREAM_ID, positionBytes);

		if(positionBytes.size() != POSITION_BYTES){
			DBG_PRINTF("Position stream is %u bytes, expected %u -- dropping this round.\n",
			           static_cast<unsigned>(positionBytes.size()), static_cast<unsigned>(POSITION_BYTES));
			gotGrid = true;
			return;
		}

		// First 4 bytes of the values stream are [rows:2][cols:2] -- see the
		// header comment on grid messages. Not a real ARQ concern (a
		// truncated/corrupted values stream would already have failed CRC
		// per-fragment and never reached isComplete()), just defends against
		// a logic bug producing a nonsense shape.
		if(valuesBytesWithHeader.size() < 4){
			DBG_PRINTLN("Grid values stream too short to contain a dimensions header -- dropping this round.");
			gotGrid = true;
			return;
		}
		int gridRows = static_cast<int>(valuesBytesWithHeader[0]) | (static_cast<int>(valuesBytesWithHeader[1]) << 8);
		int gridCols = static_cast<int>(valuesBytesWithHeader[2]) | (static_cast<int>(valuesBytesWithHeader[3]) << 8);
		// Must be exactly the fixed shape, not merely a plausible one: the PC
		// leg below is header-less, so writing a differently-shaped grid there
		// would desync that leg permanently rather than just producing one bad
		// round. Drop the round instead.
		if(gridRows != GRID_ROWS || gridCols != GRID_COLS){
			DBG_PRINTF("Rover sent a %dx%d grid but this build is fixed at %dx%d -- dropping this round "
			           "(mismatched firmware on the two boards?).\n", gridRows, gridCols, GRID_ROWS, GRID_COLS);
			gotGrid = true;
			return;
		}
		std::vector<uint8_t> valuesBytes(valuesBytesWithHeader.begin() + 4, valuesBytesWithHeader.end());

		RLERuns runs;
		bool decodeOk = splitcodec::decode(valuesBytes, countsBytes, VALUE_BIT_WIDTH, runs);
		if(!decodeOk){
			DBG_PRINTLN("splitcodec::decode failed -- both streams complete but data didn't decode.");
			gotGrid = true;
			return;
		}

		std::vector<uint8_t> symbols = rleDecode(runs);
		if(symbols.size() != static_cast<size_t>(gridRows) * static_cast<size_t>(gridCols)){
			DBG_PRINTF("Decoded %u cells but header said %dx%d -- dropping this round.\n",
			           static_cast<unsigned>(symbols.size()), gridRows, gridCols);
			gotGrid = true;
			return;
		}
		DBG_PRINTF("Round %u: grid decoded, %dx%d, %u cells, %u RLE runs\n", static_cast<unsigned>(lastSeenGridRound),
		           gridRows, gridCols, static_cast<unsigned>(symbols.size()), static_cast<unsigned>(runs.values.size()));

		// Final explicit "you're done" status report -- without this, the
		// rover has no unambiguous signal that all three streams succeeded (it
		// would just eventually time out on the last resend and give up).
		std::vector<uint8_t> finalReport = encodeStatusReport(currentGridMsgId, STATUS_COMPLETE, {}, STATUS_COMPLETE, {}, STATUS_COMPLETE, {});
		sendStream(FRAGMENT_STATUS_MESSAGE_ID, finalReport, STATUS_STREAM_ID, "final fragment status (complete)");
		gotGrid = true;

		// Forward the decompressed grid + rover position to the PC over Serial:
		// [cell bytes...][x:2][y:2], little-endian, no header -- the PC-side
		// reader must read exactly GRID_ROWS*GRID_COLS + 4 bytes and stop
		// there. Every path that reaches here has already confirmed the shape,
		// so this is the only place that writes to this leg.
		Serial.write(symbols.data(), symbols.size());
		Serial.write(positionBytes.data(), positionBytes.size());
		DBG_PRINTLN("Grid + position forwarded to PC over Serial. Waiting for path instructions...");

		// Block until the PC sends back: [waypointCount:1][x:2][y:2] repeated.
		std::vector<uint8_t> countByte = readExactDataBytes(1);
		uint8_t waypointCount = countByte[0];
		std::vector<uint8_t> waypointBytes = readExactDataBytes(static_cast<size_t>(waypointCount) * 4);
		std::vector<Waypoint> path = decodeWaypoints(waypointBytes);
		DBG_PRINTF("Received %u waypoint(s) from the PC.\n", static_cast<unsigned>(path.size()));

		uint16_t currentInstructionsMsgId = instructionsMessageIdForRound(lastSeenGridRound);
		std::vector<uint8_t> instructionsBytes = encodeWaypoints(path);

		// Send, then wait for rover's ack; resend on silence up to
		// INSTRUCTIONS_MAX_RETRIES times. Real ack/retry rather than blind
		// redundancy, now that a wrong guess about how many resends are
		// "enough" isn't the only thing standing between a lost packet and a
		// stuck round.
		bool acked = false;
		for(int attempt = 0; !acked && attempt <= INSTRUCTIONS_MAX_RETRIES; attempt++){
			if(attempt == 0){
				DBG_PRINTLN("Sending instructions back to rover...");
			} else {
				DBG_PRINTF("No ack yet -- resending instructions (attempt %d/%d)...\n", attempt, INSTRUCTIONS_MAX_RETRIES);
			}
			sendStream(currentInstructionsMsgId, instructionsBytes, INSTRUCTIONS_STREAM_ID, "instructions");

			unsigned long ackWaitStart = millis();
			while(millis() - ackWaitStart < INSTRUCTIONS_ACK_TIMEOUT_MS){
				std::vector<uint8_t> ackRaw;
				if(transport.receive(ackRaw)){
					packetizer::Packet ackPacket;
					uint16_t subject;
					if(packetizer::deserialize(ackRaw, ackPacket) && ackPacket.header.messageId == INSTRUCTIONS_ACK_MESSAGE_ID
					   && decodeInstructionsAck(ackPacket.payload, subject) && subject == currentInstructionsMsgId){
						acked = true;
						break;
					}
				}
			}
		}
		if(!acked){
			DBG_PRINTLN("Never got an ack for this round's instructions -- rover may not have received them.");
		}
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
			valuesMissing = reassembler.missingFragments(currentGridMsgId, VALUES_STREAM_ID);
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
			countsMissing = reassembler.missingFragments(currentGridMsgId, COUNTS_STREAM_ID);
			countsStatus = countsMissing.empty() ? STATUS_NOTHING_RECEIVED : STATUS_PARTIAL;
		}

		uint8_t positionStatus;
		std::vector<uint16_t> positionMissing;
		if(positionComplete){
			positionStatus = STATUS_COMPLETE;
		} else {
			positionMissing = reassembler.missingFragments(currentGridMsgId, POSITION_STREAM_ID);
			positionStatus = positionMissing.empty() ? STATUS_NOTHING_RECEIVED : STATUS_PARTIAL;
		}

		DBG_PRINTLN("Quiet period elapsed with the grid still incomplete -- sending fragment-status report.");
		std::vector<uint8_t> reportBytes = encodeStatusReport(currentGridMsgId, valuesStatus, valuesMissing,
		                                                      countsStatus, countsMissing, positionStatus, positionMissing);
		sendStream(FRAGMENT_STATUS_MESSAGE_ID, reportBytes, STATUS_STREAM_ID, "fragment status");
		reportSentForThisQuietPeriod = true;
	}
}

#else
	#error "Build with -D DEVICE_ROLE_ROVER or -D DEVICE_ROLE_BASE (see platformio.ini environments)"
#endif
