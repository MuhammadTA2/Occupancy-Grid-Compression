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

// THEORETICAL two-way hardware test -- BASE role. See
// two_way_rover_main.cpp for the overall design and two_way_protocol.h for
// the shared wire format/timing.
//
// Base defaults to listening. A rolling quiet-period timer -- reset by any
// received fragment, and also by resolving a round -- fires after
// BASE_QUIET_PERIOD_MS of silence. At that point base evaluates whatever
// it has accumulated (if anything) for the most recent round it has seen
// and broadcasts a response: OK-with-canned-path if a complete, plausible
// round was received, bad-data/rescan otherwise. If base has never learned
// a round at all (total silence since boot or since the last resolve), it
// broadcasts the same bad-data signal under a fixed wildcard messageId,
// since there's no round number yet to address a response to. The
// response repeats for BASE_RESPONSE_DURATION_MS -- base intentionally
// stops listening during this window -- then it returns to listening.
//
//   pio run -e lora_test_two_way_base -t upload

namespace{
    LoRaTransport transport;
    PacketReassembler reassembler;

    uint16_t lastSeenRound = 0;       // 0 = no round tracked right now
    unsigned long lastFragmentMillis = 0; // last time anything reset the quiet clock

    // Canned stand-in path -- same as test_scripts/simulate_basestation.py's
    // FAKE_PATH. Real path planning is a separate, not-yet-built piece.
    const std::vector<Waypoint> FAKE_PATH = { {2, 2}, {5, 8}, {10, 15}, {18, 18} };

    bool receiveOne(packetizer::Packet& out){
        std::vector<uint8_t> raw;
        if(!transport.receive(raw)) return false;
        if(LoRa.packetRssi() < RSSI_FLOOR_DBM) return false;
        return packetizer::deserialize(raw, out);
    }

    // Repeats the (possibly multi-fragment) response for
    // BASE_RESPONSE_DURATION_MS -- relies on PacketReassembler's
    // duplicate-tolerant union on the rover side, so it doesn't matter
    // which repetition rover's listen windows happen to catch.
    void broadcastResponse(uint16_t messageId, const std::vector<uint8_t>& payload){
        std::vector<packetizer::Packet> packets = packetizer::fragment(messageId, RESPONSE_STREAM_ID, payload);
        unsigned long start = millis();
        while(millis() - start < BASE_RESPONSE_DURATION_MS){
            for(const auto& p : packets){
                if(millis() - start >= BASE_RESPONSE_DURATION_MS) break;
                transport.send(packetizer::serialize(p));
            }
        }
    }
}

void setup(){
    Serial.begin(115200);
    delay(1000);

    LoRa.setPins(LORA_PIN_NSS, LORA_PIN_RST, LORA_PIN_DIO0);
    if(!LoRa.begin(LORA_FREQUENCY_HZ)){
        while(true) delay(1000);
    }
    LoRa.enableCrc();
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
}

void loop(){
    packetizer::Packet packet;
    if(receiveOne(packet)){
        if(isGridMsgId(packet.header.messageId)){
            uint16_t round = roundOfGridMsgId(packet.header.messageId);
            if(round > lastSeenRound){
                // Genuinely new round -- old reassembler entries for the
                // previous round just sit unused (round-scoped IDs mean
                // they can never be mistaken for this round's data).
                lastSeenRound = round;
            }
            if(round == lastSeenRound){
                reassembler.receive(packet);
                lastFragmentMillis = millis();
            }
            // round < lastSeenRound: stale straggler from an
            // already-resolved round -- ignored, doesn't move tracking
            // backward mid-round.
        }
        // anything else (e.g. this base's own broadcast looping back,
        // which shouldn't happen but is harmless if it did) -- ignored
    }

    // How long since anything last reset the quiet clock. lastFragmentMillis
    // stays 0 only before the very first event of the whole session (boot,
    // nothing received or resolved yet) -- in that case count from boot
    // (millis()) so the wildcard rescan still fires on schedule rather than
    // never; every subsequent reset sets a real timestamp.
    unsigned long quietFor = (lastFragmentMillis == 0) ? millis() : (millis() - lastFragmentMillis);
    if(quietFor < BASE_QUIET_PERIOD_MS) return;

    if(lastSeenRound == 0){
        // Never learned a round at all -- no ID to address a response to.
        broadcastResponse(WILDCARD_RESCAN_MESSAGE_ID, encodeBadResponse());
        lastFragmentMillis = millis();
        return;
    }

    uint16_t gridMsgId = roundToGridMsgId(lastSeenRound);
    uint16_t responseMsgId = roundToResponseMsgId(lastSeenRound);
    bool valuesComplete = reassembler.isComplete(gridMsgId, VALUES_STREAM_ID);
    bool countsComplete = reassembler.isComplete(gridMsgId, COUNTS_STREAM_ID);
    bool positionComplete = reassembler.isComplete(gridMsgId, POSITION_STREAM_ID);

    bool ok = false;
    if(valuesComplete && countsComplete && positionComplete){
        std::vector<uint8_t> valuesBytesWithHeader, countsBytes, positionBytes;
        reassembler.tryGetCompleteStream(gridMsgId, VALUES_STREAM_ID, valuesBytesWithHeader);
        reassembler.tryGetCompleteStream(gridMsgId, COUNTS_STREAM_ID, countsBytes);
        reassembler.tryGetCompleteStream(gridMsgId, POSITION_STREAM_ID, positionBytes);

        int16_t posX = 0, posY = 0;
        if(valuesBytesWithHeader.size() >= 4 && decodePosition(positionBytes, posX, posY) && isPlausiblePosition(posX, posY)){
            int gridRows = static_cast<int>(valuesBytesWithHeader[0]) | (static_cast<int>(valuesBytesWithHeader[1]) << 8);
            int gridCols = static_cast<int>(valuesBytesWithHeader[2]) | (static_cast<int>(valuesBytesWithHeader[3]) << 8);
            if(gridRows > 0 && gridCols > 0 && static_cast<size_t>(gridRows) * static_cast<size_t>(gridCols) <= MAX_GRID_CELLS){
                std::vector<uint8_t> valuesBytes(valuesBytesWithHeader.begin() + 4, valuesBytesWithHeader.end());
                RLERuns runs;
                if(splitcodec::decode(valuesBytes, countsBytes, VALUE_BIT_WIDTH, runs)){
                    std::vector<uint8_t> symbols = rleDecode(runs);
                    if(symbols.size() == static_cast<size_t>(gridRows) * static_cast<size_t>(gridCols)){
                        ok = true;
                    }
                }
            }
        }
    }

    broadcastResponse(responseMsgId, ok ? encodeGoodResponse(FAKE_PATH) : encodeBadResponse());

    // Round resolved (good or bad) -- back to default listening for the next round.
    lastSeenRound = 0;
    lastFragmentMillis = millis();
}
