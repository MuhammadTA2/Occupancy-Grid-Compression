#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "pins.h"
#include "grid.h"
#include "rle.h"
#include "split_codec.h"
#include "packetizer.h"
#include "packet_reassembler.h"
#include "lora_transport.h"
#include "coder_type.h"
#include "metrics.h"

using namespace compressor;

// One-way real-RF hardware test: flash this to the board WITHOUT a working
// antenna (receive-only -- never transmits, so it's safe for a damaged/no
// antenna board). Pairs with grid_sender_main.cpp on the antenna board.
// Listens, reassembles via the real PacketReassembler, decodes via the real
// splitcodec/rle codec, and prints a full report. Capture that report to a
// file from the PC side with, e.g.:
//   pio device monitor -p COMx -b 115200 --filter log2file
//
//   pio run -e lora_test_grid_receiver -t upload

namespace{
    const uint16_t GRID_TEST_MESSAGE_ID = 9001;
    const uint8_t VALUES_STREAM_ID = 0;
    const uint8_t COUNTS_STREAM_ID = 1;
    const uint8_t VALUE_BIT_WIDTH = 2;

    LoRaTransport transport;
    PacketReassembler reassembler;
    bool done = false;

    const char* coderTypeName(uint8_t raw){
        switch(static_cast<CoderType>(raw)){
            case CoderType::FixedWidth: return "FixedWidth";
            case CoderType::Varint:     return "Varint";
            case CoderType::Rice:       return "Rice";
            default:                    return "Unknown";
        }
    }

    void printGrid(const std::vector<uint8_t>& symbols, int rows, int cols){
        for(int r = 0; r < rows; r++){
            String line;
            for(int c = 0; c < cols; c++){
                line += String(symbols[static_cast<size_t>(r) * cols + c]);
            }
            Serial.println(line);
        }
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.println("LoRa grid test: RECEIVER");

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if(!LoRa.begin(LORA_FREQUENCY_HZ)){
        Serial.println("LoRa.begin() FAILED -- check wiring and antenna.");
        while(true) delay(1000);
    }
    LoRa.enableCrc();
    Serial.println("LoRa.begin() OK. Waiting for the grid test message...");
}

void loop(){
    if(done){
        delay(1000);
        return;
    }

    std::vector<uint8_t> raw;
    if(transport.receive(raw)){
        packetizer::Packet packet;
        if(!packetizer::deserialize(raw, packet)){
            Serial.println("Dropped a corrupted/malformed packet (CRC or framing check failed).");
        } else if(packet.header.messageId == GRID_TEST_MESSAGE_ID){
            reassembler.receive(packet);
            Serial.printf("Got streamId=%u fragment %u/%u, RSSI=%d\n",
                           packet.header.streamId,
                           static_cast<unsigned>(packet.header.fragmentIndex + 1),
                           static_cast<unsigned>(packet.header.totalFragments), LoRa.packetRssi());
        } else {
            Serial.printf("Ignored packet with unexpected messageId=%u\n", packet.header.messageId);
        }
    }

    bool valuesComplete = reassembler.isComplete(GRID_TEST_MESSAGE_ID, VALUES_STREAM_ID);
    bool countsComplete = reassembler.isComplete(GRID_TEST_MESSAGE_ID, COUNTS_STREAM_ID);
    if(!valuesComplete || !countsComplete) return;

    std::vector<uint8_t> valuesBytesWithHeader, countsBytes;
    reassembler.tryGetCompleteStream(GRID_TEST_MESSAGE_ID, VALUES_STREAM_ID, valuesBytesWithHeader);
    reassembler.tryGetCompleteStream(GRID_TEST_MESSAGE_ID, COUNTS_STREAM_ID, countsBytes);

    Serial.println("=== Both streams complete -- decoding ===");
    if(valuesBytesWithHeader.size() < 4){
        Serial.println("RESULT: FAIL -- values stream too short for a dimensions header.");
        done = true;
        return;
    }
    int rows = static_cast<int>(valuesBytesWithHeader[0]) | (static_cast<int>(valuesBytesWithHeader[1]) << 8);
    int cols = static_cast<int>(valuesBytesWithHeader[2]) | (static_cast<int>(valuesBytesWithHeader[3]) << 8);
    Serial.printf("Header says grid is %dx%d\n", rows, cols);
    if(rows <= 0 || cols <= 0 || static_cast<size_t>(rows) * static_cast<size_t>(cols) > 40000){
        Serial.println("RESULT: FAIL -- implausible dimensions.");
        done = true;
        return;
    }
    std::vector<uint8_t> valuesBytes(valuesBytesWithHeader.begin() + 4, valuesBytesWithHeader.end());

    RLERuns runs;
    if(!splitcodec::decode(valuesBytes, countsBytes, VALUE_BIT_WIDTH, runs)){
        Serial.println("RESULT: FAIL -- splitcodec::decode rejected the data.");
        done = true;
        return;
    }
    std::vector<uint8_t> symbols = rleDecode(runs);
    if(symbols.size() != static_cast<size_t>(rows) * static_cast<size_t>(cols)){
        Serial.printf("RESULT: FAIL -- decoded %u cells, expected %d.\n",
                       static_cast<unsigned>(symbols.size()), rows * cols);
        done = true;
        return;
    }

    Serial.printf("RESULT: OK -- decoded %u cells, %u RLE runs.\n",
                   static_cast<unsigned>(symbols.size()), static_cast<unsigned>(runs.values.size()));

    // Same stat, computed independently from what was actually received and
    // decoded here -- compare this line against the sender's "Compression:"
    // line. Matching values/counts sizes and ratio is a content-level
    // corruption check on top of the CRC checks already happening at the
    // radio and packetizer layers (see grid_sender_main.cpp's comment).
    size_t originalBytes = symbols.size();
    size_t compressedBytes = valuesBytes.size() + countsBytes.size();
    double ratio = compressionRatio(compressedBytes, originalBytes);
    Serial.printf("Compression: original=%u bytes, compressed=%u bytes (values=%u, counts=%u), ratio=%.3f:1, counts coder=%s\n",
                   static_cast<unsigned>(originalBytes), static_cast<unsigned>(compressedBytes),
                   static_cast<unsigned>(valuesBytes.size()), static_cast<unsigned>(countsBytes.size()),
                   ratio, coderTypeName(countsBytes.empty() ? 255 : countsBytes[0]));

    Serial.println("Decoded grid (0=free, 1=obstacle, 2=uncertain):");
    printGrid(symbols, rows, cols);
    Serial.println("=== Done. Reset both boards to run again. ===");
    done = true;
}
