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

namespace{
	const uint16_t MESSAGE_ID = 1;
	const uint8_t VALUES_STREAM_ID = 0;
	const uint8_t COUNTS_STREAM_ID = 1;
	const uint8_t VALUE_BIT_WIDTH = 2; // occupancy values are 0/1/2, 11 reserved

	LoRaTransport transport;
	PacketReassembler reassembler;
	bool decodedAlready = false;

	// Small synthetic occupancy grid -- kept modest on purpose. LoRa's
	// airtime per packet (tens to hundreds of ms depending on spreading
	// factor) makes a 100x100 grid like the host demo impractical for a
	// first real-hardware smoke test; 20x20 is enough to exercise multiple
	// fragments on both streams without a multi-second transmission.
	Grid buildTestGrid(){
		Grid grid = createGrid(20, 20);
		for(int i = 0; i < grid.cols; i++){
			grid.data[0 * grid.cols + i] = 1;
			grid.data[(grid.rows - 1) * grid.cols + i] = 1;
		}
		for(int r = 0; r < grid.rows; r++){
			grid.data[r * grid.cols + 0] = 1;
			grid.data[r * grid.cols + (grid.cols - 1)] = 1;
		}
		for(int r = 3; r < 17; r++){
			grid.data[r * grid.cols + 10] = 1;
		}
		for(int r = 8; r < 12; r++){
			grid.data[r * grid.cols + 10] = 0; // a gap in the wall
		}
		return grid;
	}

	void sendStream(const std::vector<uint8_t>& data, uint8_t streamId, const char* label){
		std::vector<packetizer::Packet> packets = packetizer::fragment(MESSAGE_ID, streamId, data);
		Serial.printf("Sending %s stream: %u bytes -> %u fragment(s)\n", label,
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
	}

	void runSenderOnce(){
		Grid grid = buildTestGrid();
		SymbolStream stream = toSymbolStream(grid);
		RLERuns runs = rleEncode(stream.symbols);
		splitcodec::EncodedStreams streams = splitcodec::encode(runs, VALUE_BIT_WIDTH);

		Serial.printf("Grid: %dx%d, %u cells, %u RLE runs\n", grid.rows, grid.cols,
		              static_cast<unsigned>(grid.data.size()), static_cast<unsigned>(runs.values.size()));

		sendStream(streams.valuesBytes, VALUES_STREAM_ID, "values");
		sendStream(streams.countsBytes, COUNTS_STREAM_ID, "counts");

		Serial.println("Done sending. (No ARQ in this first firmware pass -- see main.cpp comment.)");
	}

	void runReceiverStep(){
		std::vector<uint8_t> raw;
		if(!transport.receive(raw)) return;

		packetizer::Packet packet;
		if(!packetizer::deserialize(raw, packet)){
			Serial.println("Received a corrupted/malformed packet -- dropped.");
			return;
		}

		reassembler.receive(packet);
		Serial.printf("Got messageId=%u streamId=%u fragment %u/%u (%u byte payload), RSSI=%d\n",
		              packet.header.messageId, packet.header.streamId,
		              static_cast<unsigned>(packet.header.fragmentIndex + 1),
		              static_cast<unsigned>(packet.header.totalFragments),
		              static_cast<unsigned>(packet.payload.size()), LoRa.packetRssi());

		if(decodedAlready) return;
		bool valuesComplete = reassembler.isComplete(MESSAGE_ID, VALUES_STREAM_ID);
		bool countsComplete = reassembler.isComplete(MESSAGE_ID, COUNTS_STREAM_ID);
		if(!valuesComplete || !countsComplete) return;

		std::vector<uint8_t> valuesBytes, countsBytes;
		reassembler.tryGetCompleteStream(MESSAGE_ID, VALUES_STREAM_ID, valuesBytes);
		reassembler.tryGetCompleteStream(MESSAGE_ID, COUNTS_STREAM_ID, countsBytes);

		RLERuns runs;
		bool decodeOk = splitcodec::decode(valuesBytes, countsBytes, VALUE_BIT_WIDTH, runs);
		if(!decodeOk){
			Serial.println("splitcodec::decode failed -- both streams complete but data didn't decode.");
			decodedAlready = true;
			return;
		}

		std::vector<uint8_t> symbols = rleDecode(runs);
		SymbolStream stream;
		stream.format = StreamFormat::Raw;
		stream.symbols = symbols;
		Grid rebuilt = fromSymbolStream(stream, 20, 20);

		Serial.printf("Decode succeeded: %u cells reconstructed, %u RLE runs\n",
		              static_cast<unsigned>(rebuilt.data.size()), static_cast<unsigned>(runs.values.size()));
		decodedAlready = true;
	}
}

void setup(){
	Serial.begin(115200);
	delay(1000); // give the serial monitor time to attach

	LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
	if(!LoRa.begin(LORA_FREQUENCY_HZ)){
		Serial.println("LoRa.begin() failed -- check wiring and LORA_FREQUENCY_HZ (pins.h).");
		while(true) delay(1000);
	}
	Serial.println("LoRa radio initialized.");

#if defined(DEVICE_ROLE_SENDER)
	Serial.println("Role: SENDER");
	runSenderOnce();
#elif defined(DEVICE_ROLE_RECEIVER)
	Serial.println("Role: RECEIVER -- waiting for packets...");
#else
	#error "Build with -D DEVICE_ROLE_SENDER or -D DEVICE_ROLE_RECEIVER (see platformio.ini environments)"
#endif
}

void loop(){
#if defined(DEVICE_ROLE_RECEIVER)
	runReceiverStep();
#else
	delay(1000); // sender has nothing left to do after setup()'s one-shot send
#endif
}
