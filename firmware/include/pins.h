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

// Radio tuning shared by both ends of a link -- centralized here (rather
// than duplicated as literals in each sketch) specifically so the sender and
// receiver can't drift out of sync with each other. Spreading factor in
// particular MUST match on both sides or they simply won't hear each other,
// same failure mode as a frequency mismatch above.
//   LORA_TX_POWER_DBM: max is +20 (SX127x PA_BOOST); library default is +17.
//   LORA_SPREADING_FACTOR: 6-12, library default is 7 (fastest, least
//     sensitive). Raising it trades airtime (slower) for real receiver
//     sensitivity margin -- useful for range testing.
constexpr int LORA_TX_POWER_DBM = 20;
constexpr int LORA_SPREADING_FACTOR = 7;

// Second hardware UART, intended for a dedicated binary Jetson/computer data
// link (grid bytes in on ROVER, grid+waypoint bytes out/in on BASE) wired to
// an external USB-to-serial adapter, kept separate from the USB `Serial`
// connection so debug text and binary payloads never share one wire.
//
// NOT currently used by main.cpp: this bench-test setup has both ESP32s
// reaching the PC only through their own programming-port USB, so the data
// protocol runs over `Serial` instead (with debug prints compiled out --
// see ENABLE_SERIAL_DEBUG in main.cpp). These constants are kept for when a
// real external adapter (or the real Jetson) is wired up here again.
//
//   Jetson/computer pin -> ESP32 GPIO
//   RX (their TX)        -> GPIO 17 (DATA_UART_TX_PIN, ESP32's TX)
//   TX (their RX)        -> GPIO 16 (DATA_UART_RX_PIN, ESP32's RX)
//   GND                  -> GND (common ground is required)
constexpr int DATA_UART_RX_PIN = 16;
constexpr int DATA_UART_TX_PIN = 17;
constexpr long DATA_UART_BAUD = 115200;

#endif
