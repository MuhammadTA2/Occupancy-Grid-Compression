#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "pins.h"
#include "packetizer.h"
#include "packet_reassembler.h"
#include "lora_transport.h"

using namespace compressor;

// One-way real-RF hardware test: flash this to the board WITHOUT a working
// antenna (receive-only -- never transmits). Pairs with
// instructions_sender_main.cpp on the antenna board.
//   pio device monitor -p COMx -b 115200 --filter log2file
//
//   pio run -e lora_test_instructions_receiver -t upload

namespace{
    const uint16_t INSTRUCTIONS_TEST_MESSAGE_ID = 9002;
    const uint8_t INSTRUCTIONS_STREAM_ID = 0;
    const size_t MAX_PLAUSIBLE_WAYPOINTS = 64; // same sanity cap as main.cpp's rover

    LoRaTransport transport;
    PacketReassembler reassembler;
    bool done = false;
}

void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.println("LoRa instructions test: RECEIVER");

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if(!LoRa.begin(LORA_FREQUENCY_HZ)){
        Serial.println("LoRa.begin() FAILED -- check wiring and antenna.");
        while(true) delay(1000);
    }
    LoRa.enableCrc();
    Serial.println("LoRa.begin() OK. Waiting for the instructions test message...");
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
        } else if(packet.header.messageId == INSTRUCTIONS_TEST_MESSAGE_ID){
            reassembler.receive(packet);
            Serial.printf("Got fragment %u/%u, RSSI=%d\n",
                           static_cast<unsigned>(packet.header.fragmentIndex + 1),
                           static_cast<unsigned>(packet.header.totalFragments), LoRa.packetRssi());
        } else {
            Serial.printf("Ignored packet with unexpected messageId=%u\n", packet.header.messageId);
        }
    }

    if(!reassembler.isComplete(INSTRUCTIONS_TEST_MESSAGE_ID, INSTRUCTIONS_STREAM_ID)) return;

    std::vector<uint8_t> bytes;
    reassembler.tryGetCompleteStream(INSTRUCTIONS_TEST_MESSAGE_ID, INSTRUCTIONS_STREAM_ID, bytes);
    Serial.println("=== Stream complete -- decoding ===");

    if(bytes.size() % 4 != 0 || bytes.size() / 4 > MAX_PLAUSIBLE_WAYPOINTS){
        Serial.printf("RESULT: FAIL -- implausible payload (%u bytes).\n", static_cast<unsigned>(bytes.size()));
        done = true;
        return;
    }

    size_t count = bytes.size() / 4;
    Serial.printf("RESULT: OK -- %u waypoint(s):\n", static_cast<unsigned>(count));
    for(size_t i = 0; i < count; i++){
        int16_t x = static_cast<int16_t>(bytes[i * 4] | (bytes[i * 4 + 1] << 8));
        int16_t y = static_cast<int16_t>(bytes[i * 4 + 2] | (bytes[i * 4 + 3] << 8));
        Serial.printf("  (%d, %d)\n", x, y);
    }
    Serial.println("=== Done. Reset both boards to run again. ===");
    done = true;
}
