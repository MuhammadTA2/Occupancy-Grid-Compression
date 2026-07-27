#ifndef PINS_H
#define PINS_H

// Standard ESP32 DevKitC <-> RFM95/SX127x wiring (VSPI bus). Change these if
// your kit's silkscreen uses different GPIO numbers -- these are the common
// defaults used by most ESP32+RFM95 tutorials/libraries, not something the
// hardware enforces.
//
//   RFM95 pin  -> ESP32 GPIO
//   VIN        -> 3V3   (NOT 5V -- the SX1276 is 3.3V-only, 5V will damage it)
//   GND        -> GND
//   SCK        -> GPIO 18
//   MISO       -> GPIO 19
//   MOSI       -> GPIO 23
//   NSS (CS)   -> GPIO 5
//   RST        -> GPIO 14
//   DIO0       -> GPIO 2

constexpr int LORA_PIN_NSS  = 5;
constexpr int LORA_PIN_RST  = 14;
constexpr int LORA_PIN_DIO0 = 2;

// SX1276-family radios are tunable across roughly 862-1020 MHz regardless of
// which "band" (868/900/915) the module is nominally sold as -- what matters
// is that BOTH boards use the exact same value here, and that the value is
// legal to transmit on on your hardware/region. Adjust if needed; this is
// the one setting that must match on both the sender and receiver builds.
constexpr long LORA_FREQUENCY_HZ = 915E6;

#endif
