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
#include "two_way_protocol.h"

using namespace compressor;
using namespace twoway;

// THEORETICAL two-way hardware test -- ROVER role. Needs a working antenna
// on BOTH ends to actually run end to end (this side transmits and
// receives); can't be fully validated until the second antenna is fixed.
// Written now per the design discussion so it's ready when that happens.
// See two_way_protocol.h for the shared wire format/timing and
// two_way_base_main.cpp for the other side.
//
// One round: read a grid ([rows:2][cols:2][cells...]) plus a position
// ([x:2][y:2], direct bit-packed, no compression) from Jetson over Serial,
// fragment and send both over LoRa, then oscillate 500ms send/500ms listen
// -- continuing to send unsent fragments from where the cursor left off
// each send window, not restarting from fragment 1 -- until base responds
// (a path, or bad-data/rescan) or ROVER_GIVEUP_MS elapses. Report the
// outcome to Jetson, then wait for the next capture.
//
//   pio run -e lora_test_two_way_rover -t upload

namespace{
    LoRaTransport transport;
    PacketReassembler reassembler;
    uint16_t roundNumber = 0;

    // Blocks (yielding via delay(), not busy-spinning) until exactly `count`
    // bytes have arrived on Serial -- the native-USB link doubling as the
    // binary Jetson data channel, same convention main.cpp uses.
    std::vector<uint8_t> readExactDataBytes(size_t count){
        std::vector<uint8_t> buf;
        buf.reserve(count);
        while(buf.size() < count){
            if(Serial.available()) buf.push_back(static_cast<uint8_t>(Serial.read()));
            else delay(1);
        }
        return buf;
    }

    // Receives at most one packet, applying the RSSI floor on top of the
    // CRC checks already enforced at the radio and packetizer layers.
    bool receiveOne(packetizer::Packet& out){
        std::vector<uint8_t> raw;
        if(!transport.receive(raw)) return false;
        if(LoRa.packetRssi() < RSSI_FLOOR_DBM) return false; // treat as noise, drop silently
        return packetizer::deserialize(raw, out);
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000); // let the PC-side script attach after the USB-open auto-reset

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if(!LoRa.begin(LORA_FREQUENCY_HZ)){
        // No separate debug channel here -- Serial carries Jetson's binary
        // protocol, same constraint as main.cpp's ENABLE_SERIAL_DEBUG note.
        while(true) delay(1000);
    }
    LoRa.enableCrc();
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
}

void loop(){
    roundNumber++;
    uint16_t gridMsgId = roundToGridMsgId(roundNumber);

    // --- Read this round's data from Jetson ---
    int gridRows = 0, gridCols = 0;
    while(true){
        std::vector<uint8_t> header = readExactDataBytes(4);
        int r = static_cast<int>(header[0]) | (static_cast<int>(header[1]) << 8);
        int c = static_cast<int>(header[2]) | (static_cast<int>(header[3]) << 8);
        if(r > 0 && c > 0 && static_cast<size_t>(r) * static_cast<size_t>(c) <= MAX_GRID_CELLS){
            gridRows = r; gridCols = c;
            break;
        }
        // implausible header (corrupted/desynced read) -- keep waiting for a real one
    }
    std::vector<uint8_t> cellBytes = readExactDataBytes(static_cast<size_t>(gridRows) * gridCols);
    std::vector<uint8_t> positionBytes = readExactDataBytes(4);
    int16_t posX = 0, posY = 0;
    decodePosition(positionBytes, posX, posY);

    // --- Build this round's fragment list: values, then counts, then position ---
    Grid grid = rebuildGrid(cellBytes, gridRows, gridCols);
    SymbolStream stream = toSymbolStream(grid);
    RLERuns runs = rleEncode(stream.symbols);
    splitcodec::EncodedStreams streams = splitcodec::encode(runs, VALUE_BIT_WIDTH);

    std::vector<uint8_t> valuesPayload;
    valuesPayload.reserve(4 + streams.valuesBytes.size());
    valuesPayload.push_back(static_cast<uint8_t>(gridRows & 0xFF));
    valuesPayload.push_back(static_cast<uint8_t>((gridRows >> 8) & 0xFF));
    valuesPayload.push_back(static_cast<uint8_t>(gridCols & 0xFF));
    valuesPayload.push_back(static_cast<uint8_t>((gridCols >> 8) & 0xFF));
    valuesPayload.insert(valuesPayload.end(), streams.valuesBytes.begin(), streams.valuesBytes.end());

    std::vector<packetizer::Packet> allFragments;
    for(auto& p : packetizer::fragment(gridMsgId, VALUES_STREAM_ID, valuesPayload)) allFragments.push_back(p);
    for(auto& p : packetizer::fragment(gridMsgId, COUNTS_STREAM_ID, streams.countsBytes)) allFragments.push_back(p);
    for(auto& p : packetizer::fragment(gridMsgId, POSITION_STREAM_ID, encodePosition(posX, posY))) allFragments.push_back(p);

    // --- Oscillate 500ms send/listen. Send is continue-from-cursor (never
    //     restarts from fragment 0), relying on round-level pass/fail --
    //     not per-fragment ARQ -- as the only loss-recovery mechanism. ---
    size_t cursor = 0;
    unsigned long roundStart = millis();
    bool gotResponse = false;
    bool badData = false;
    std::vector<Waypoint> waypoints;

    while(!gotResponse && millis() - roundStart < ROVER_GIVEUP_MS){
        unsigned long windowStart = millis();
        while(cursor < allFragments.size() && millis() - windowStart < ROVER_CYCLE_MS){
            transport.send(packetizer::serialize(allFragments[cursor]));
            cursor++;
        }

        windowStart = millis();
        while(millis() - windowStart < ROVER_CYCLE_MS){
            packetizer::Packet packet;
            if(!receiveOne(packet)) continue;

            if(packet.header.messageId == WILDCARD_RESCAN_MESSAGE_ID){
                gotResponse = true;
                badData = true;
                break;
            }
            if(isResponseMsgId(packet.header.messageId) && roundOfResponseMsgId(packet.header.messageId) == roundNumber){
                reassembler.receive(packet);
                uint16_t responseMsgId = roundToResponseMsgId(roundNumber);
                if(reassembler.isComplete(responseMsgId, RESPONSE_STREAM_ID)){
                    std::vector<uint8_t> responseBytes;
                    reassembler.tryGetCompleteStream(responseMsgId, RESPONSE_STREAM_ID, responseBytes);
                    if(decodeResponse(responseBytes, badData, waypoints)) gotResponse = true;
                    break;
                }
            }
            // anything else (stale round, unrelated messageId) -- ignored
        }
    }

    // --- Report outcome to Jetson: [count:1][x:2][y:2]*count, or a single
    //     sentinel byte -- same wire shape base used, forwarded verbatim ---
    if(!gotResponse){
        Serial.write(GAVE_UP_SENTINEL); // rover's own give-up, distinct from base explicitly saying bad data
    } else if(badData){
        Serial.write(BAD_DATA_SENTINEL); // base says rescan
    } else {
        std::vector<uint8_t> out = encodeGoodResponse(waypoints);
        Serial.write(out.data(), out.size());
    }
}
