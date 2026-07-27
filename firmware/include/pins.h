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

// Must match the modules' actual operating frequency, and must be identical
// on both the sender and receiver builds -- a mismatch here means the two
// radios simply won't hear each other, with no error reported by either side.
constexpr long LORA_FREQUENCY_HZ = 900E6;

// Second hardware UART, dedicated to the binary Jetson/computer data link
// (grid bytes in on ROVER, grid+waypoint bytes out/in on BASE). Kept
// separate from the USB `Serial` connection, which carries only
// human-readable debug text -- mixing both on one line would make binary
// payloads unparseable on the Jetson/computer side (debug text interleaved
// with raw bytes, no way to tell them apart).
//
//   Jetson/computer pin -> ESP32 GPIO
//   RX (their TX)        -> GPIO 17 (DATA_UART_TX_PIN, ESP32's TX)
//   TX (their RX)        -> GPIO 16 (DATA_UART_RX_PIN, ESP32's RX)
//   GND                  -> GND (common ground is required)
constexpr int DATA_UART_RX_PIN = 16;
constexpr int DATA_UART_TX_PIN = 17;
constexpr long DATA_UART_BAUD = 115200;

#endif
