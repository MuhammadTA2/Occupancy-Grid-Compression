#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "pins.h"

// Standalone LoRa hardware test -- not part of the project protocol. Flash
// this to one board and lora_test_sender.cpp's build to the other, to
// confirm the two radios can hear each other at the RF level before
// debugging anything in main.cpp.
//
//   pio run -e lora_test_receiver -t upload

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("LoRa hardware test: RECEIVER");

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("LoRa.begin() FAILED -- check wiring (see pins.h) and antenna.");
        while (true) delay(1000);
    }
    Serial.println("LoRa.begin() OK. Waiting for packets...");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        Serial.print("Received packet '");
        while (LoRa.available()) {
            Serial.print((char)LoRa.read());
        }
        Serial.print("' with RSSI ");
        Serial.println(LoRa.packetRssi());
    }
}
