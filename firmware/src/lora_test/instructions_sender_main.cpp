#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "pins.h"
#include "packetizer.h"
#include "lora_transport.h"

using namespace compressor;

// One-way real-RF hardware test: flash this to the board that HAS a working
// antenna (keep the antenna board as sender for this leg too, per the
// grid_sender/receiver test's reasoning -- only the antenna side should ever
// transmit). Sends a premade canned waypoint list once, then halts. Pairs
// with instructions_receiver_main.cpp on the antenna-less board.
//
//   pio run -e lora_test_instructions_sender -t upload

namespace{
    const uint16_t INSTRUCTIONS_TEST_MESSAGE_ID = 9002;
    const uint8_t INSTRUCTIONS_STREAM_ID = 0;

    LoRaTransport transport;

    struct Waypoint{ int16_t x; int16_t y; };
    // Same canned path as test_scripts/simulate_basestation.py's FAKE_PATH.
    const Waypoint FAKE_PATH[] = { {2, 2}, {5, 8}, {10, 15}, {18, 18} };

    std::vector<uint8_t> encodeWaypoints(){
        std::vector<uint8_t> bytes;
        for(const Waypoint& w : FAKE_PATH){
            bytes.push_back(static_cast<uint8_t>(w.x & 0xFF));
            bytes.push_back(static_cast<uint8_t>((w.x >> 8) & 0xFF));
            bytes.push_back(static_cast<uint8_t>(w.y & 0xFF));
            bytes.push_back(static_cast<uint8_t>((w.y >> 8) & 0xFF));
        }
        return bytes;
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.println("LoRa instructions test: SENDER");

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if(!LoRa.begin(LORA_FREQUENCY_HZ)){
        Serial.println("LoRa.begin() FAILED -- check wiring and antenna.");
        while(true) delay(1000);
    }
    LoRa.enableCrc();
    Serial.println("LoRa.begin() OK.");

    std::vector<uint8_t> data = encodeWaypoints();
    std::vector<packetizer::Packet> packets = packetizer::fragment(INSTRUCTIONS_TEST_MESSAGE_ID, INSTRUCTIONS_STREAM_ID, data);
    Serial.printf("Sending instructions: %u bytes -> %u fragment(s)\n",
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
    Serial.println("Done sending. Halting -- reset the board to send again.");
}

void loop(){
    delay(1000);
}
