#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "pins.h"

// Standalone LoRa hardware test -- not part of the project protocol. Flash
// this to one board and lora_test_receiver.cpp's build to the other, to
// confirm the two radios can hear each other at the RF level before
// debugging anything in main.cpp.
//
//   pio run -e lora_test_sender -t upload

int counter = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("LoRa hardware test: SENDER");

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("LoRa.begin() FAILED -- check wiring (see pins.h) and antenna.");
        while (true) delay(1000);
    }
    Serial.println("LoRa.begin() OK. Sending a packet every 2s...");
}

void loop() {
    Serial.print("Sending packet: ");
    Serial.println(counter);

    LoRa.beginPacket();
    LoRa.print("hello ");
    LoRa.print(counter);
    LoRa.endPacket();

    counter++;
    delay(2000);
}
