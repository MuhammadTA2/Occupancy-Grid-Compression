#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "pins.h"
#include "grid.h"
#include "rle.h"
#include "split_codec.h"
#include "packetizer.h"
#include "lora_transport.h"
#include "coder_type.h"
#include "metrics.h"

using namespace compressor;

// One-way real-RF hardware test: flash this to the board that HAS a working
// antenna. Encodes a premade grid through the real production codec
// (rleEncode -> splitcodec::encode -> packetizer::fragment), sends it once
// over LoRa, then halts -- deliberately no retries/ARQ, this is meant to
// check "can a real, real-RF multi-fragment transmission survive the trip
// with a real antenna on the sending side," not exercise the loss-recovery
// logic (that needs a second working antenna on the receive side). Pair
// with grid_receiver_main.cpp on the other (antenna-less) board.
//
//   pio run -e lora_test_grid_sender -t upload

namespace{
    const uint16_t GRID_TEST_MESSAGE_ID = 9001;
    const uint8_t VALUES_STREAM_ID = 0;
    const uint8_t COUNTS_STREAM_ID = 1;
    const uint8_t VALUE_BIT_WIDTH = 2; // occupancy values are 0/1/2, 11 reserved

    const int GRID_ROWS = 20;
    const int GRID_COLS = 20;

    LoRaTransport transport;

    const char* coderTypeName(uint8_t raw){
        switch(static_cast<CoderType>(raw)){
            case CoderType::FixedWidth: return "FixedWidth";
            case CoderType::Varint:     return "Varint";
            case CoderType::Rice:       return "Rice";
            default:                    return "Unknown";
        }
    }

    // Same bordered-square-with-a-gapped-wall pattern as
    // test_scripts/simulate_jetson.py's build_test_grid() -- deterministic
    // and easy to eyeball on the receiving end.
    Grid buildTestGrid(){
        Grid grid;
        grid.rows = GRID_ROWS;
        grid.cols = GRID_COLS;
        grid.data.assign(static_cast<size_t>(GRID_ROWS) * GRID_COLS, 0);
        for(int c = 0; c < GRID_COLS; c++){
            grid.data[0 * GRID_COLS + c] = 1;
            grid.data[(GRID_ROWS - 1) * GRID_COLS + c] = 1;
        }
        for(int r = 0; r < GRID_ROWS; r++){
            grid.data[r * GRID_COLS + 0] = 1;
            grid.data[r * GRID_COLS + (GRID_COLS - 1)] = 1;
        }
        for(int r = 3; r < 17; r++) grid.data[r * GRID_COLS + 10] = 1;
        for(int r = 8; r < 12; r++) grid.data[r * GRID_COLS + 10] = 0;
        return grid;
    }

    void sendStream(uint16_t messageId, const std::vector<uint8_t>& data, uint8_t streamId, const char* label){
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
            delay(300);
        }
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.println("LoRa grid test: SENDER");

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if(!LoRa.begin(LORA_FREQUENCY_HZ)){
        Serial.println("LoRa.begin() FAILED -- check wiring and antenna.");
        while(true) delay(1000);
    }
    LoRa.enableCrc(); // reject radio-level noise falsely detected as a packet
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    Serial.printf("LoRa.begin() OK. TX power=%d dBm, SF=%d\n", LORA_TX_POWER_DBM, LORA_SPREADING_FACTOR);

    Grid grid = buildTestGrid();
    SymbolStream stream = toSymbolStream(grid);
    RLERuns runs = rleEncode(stream.symbols);
    splitcodec::EncodedStreams streams = splitcodec::encode(runs, VALUE_BIT_WIDTH);
    Serial.printf("Grid: %dx%d, %u cells, %u RLE runs\n", grid.rows, grid.cols,
                   static_cast<unsigned>(grid.data.size()), static_cast<unsigned>(runs.values.size()));

    // Compression stats -- printed here and (independently, from what it
    // actually decoded) on the receiver. If the two sides' numbers match,
    // that's a content-level corruption check on top of the CRC checks
    // already happening at the radio and packetizer layers: a decode that
    // silently produced different data would very likely also produce a
    // different RLE run structure, and therefore a different compressed
    // size/ratio here.
    size_t originalBytes = grid.data.size();
    size_t compressedBytes = streams.valuesBytes.size() + streams.countsBytes.size();
    double ratio = compressionRatio(compressedBytes, originalBytes);
    Serial.printf("Compression: original=%u bytes, compressed=%u bytes (values=%u, counts=%u), ratio=%.3f:1, counts coder=%s\n",
                   static_cast<unsigned>(originalBytes), static_cast<unsigned>(compressedBytes),
                   static_cast<unsigned>(streams.valuesBytes.size()), static_cast<unsigned>(streams.countsBytes.size()),
                   ratio, coderTypeName(streams.countsBytes.empty() ? 255 : streams.countsBytes[0]));

    // [rows:2][cols:2] prefix on the values stream -- same framing main.cpp
    // uses, so the receiver knows the grid's shape.
    std::vector<uint8_t> valuesPayload;
    valuesPayload.reserve(4 + streams.valuesBytes.size());
    valuesPayload.push_back(static_cast<uint8_t>(grid.rows & 0xFF));
    valuesPayload.push_back(static_cast<uint8_t>((grid.rows >> 8) & 0xFF));
    valuesPayload.push_back(static_cast<uint8_t>(grid.cols & 0xFF));
    valuesPayload.push_back(static_cast<uint8_t>((grid.cols >> 8) & 0xFF));
    valuesPayload.insert(valuesPayload.end(), streams.valuesBytes.begin(), streams.valuesBytes.end());

    sendStream(GRID_TEST_MESSAGE_ID, valuesPayload, VALUES_STREAM_ID, "grid values");
    sendStream(GRID_TEST_MESSAGE_ID, streams.countsBytes, COUNTS_STREAM_ID, "grid counts");
    Serial.println("Done sending. Halting -- reset the board to send again.");
}

void loop(){
    delay(1000);
}
