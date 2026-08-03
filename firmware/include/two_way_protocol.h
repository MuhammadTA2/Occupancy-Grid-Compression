#ifndef TWO_WAY_PROTOCOL_H
#define TWO_WAY_PROTOCOL_H

#include <cstdint>
#include <cstddef>
#include <vector>

// Shared by two_way_rover_main.cpp and two_way_base_main.cpp -- protocol
// constants and wire encodings that MUST be identical on both ends live
// here once, rather than as separately-typed literals in two files that
// could drift apart (same reasoning as pins.h's shared radio-tuning
// constants).
//
// THEORETICAL / not yet hardware-testable end to end: both roles here
// transmit, so this needs two working antennas to actually validate --
// written now so it's ready once that's available.
//
// No fragment-level ARQ by design: round-level pass/fail (OK-with-path vs
// bad-data-rescan) is the only loss-recovery mechanism. A lost fragment
// just means the round times out incomplete and gets redone from a fresh
// Jetson capture, not resent piecemeal -- consistent with "assume success,
// only guard against noise" rather than reimplementing the real firmware's
// fragment-status ARQ.
namespace twoway{
    // Round-scoped (like main.cpp's 08/02/2026 grid/instructions IDs) so a
    // stale reassembler entry from an old round can never make a new round
    // look falsely "complete."
    constexpr uint16_t ROUND_MESSAGE_ID_BASE = 5000;      // rover -> base: grid + position
    constexpr uint16_t RESPONSE_MESSAGE_ID_BASE = 6000;   // base -> rover: path or bad-data, for a specific round
    // Fixed (not round-scoped): used only when base has accumulated
    // *nothing* for any round and its quiet-period timer still fires --
    // there's no round number to address a response to, so rover accepts
    // this ID unconditionally as "whatever round you're on, it's bad, rescan."
    constexpr uint16_t WILDCARD_RESCAN_MESSAGE_ID = 7;

    inline uint16_t roundToGridMsgId(uint16_t round){ return static_cast<uint16_t>(ROUND_MESSAGE_ID_BASE + round); }
    inline uint16_t roundToResponseMsgId(uint16_t round){ return static_cast<uint16_t>(RESPONSE_MESSAGE_ID_BASE + round); }
    inline bool isGridMsgId(uint16_t id){ return id >= ROUND_MESSAGE_ID_BASE && id < RESPONSE_MESSAGE_ID_BASE; }
    inline uint16_t roundOfGridMsgId(uint16_t id){ return static_cast<uint16_t>(id - ROUND_MESSAGE_ID_BASE); }
    inline bool isResponseMsgId(uint16_t id){ return id >= RESPONSE_MESSAGE_ID_BASE; }
    inline uint16_t roundOfResponseMsgId(uint16_t id){ return static_cast<uint16_t>(id - RESPONSE_MESSAGE_ID_BASE); }

    constexpr uint8_t VALUES_STREAM_ID = 0;
    constexpr uint8_t COUNTS_STREAM_ID = 1;
    constexpr uint8_t POSITION_STREAM_ID = 2;
    constexpr uint8_t RESPONSE_STREAM_ID = 0;

    constexpr uint8_t VALUE_BIT_WIDTH = 2; // occupancy values 0/1/2
    constexpr size_t MAX_GRID_CELLS = 40000;

    // Timing -- see the design discussion: 500ms send/listen oscillation
    // (rover), base's 3s rolling quiet period and 3s repeat-broadcast,
    // rover's 5s overall give-up. Derived from a hand-traced airtime
    // estimate at the firmware's current SF/BW/CR, not arbitrary -- expect
    // to retune if spreading factor or grid density changes appreciably.
    constexpr unsigned long ROVER_CYCLE_MS = 500;
    constexpr unsigned long BASE_QUIET_PERIOD_MS = 3000;
    constexpr unsigned long BASE_RESPONSE_DURATION_MS = 3000;
    constexpr unsigned long ROVER_GIVEUP_MS = 5000;

    // Extra noise gate on top of the existing dual CRC (radio-level +
    // packetizer's own CRC-16) -- cheap insurance given a real
    // noise-triggered false packet was already observed on this exact
    // hardware at -118 dBm. Conservative/low on purpose: tighten once real
    // link margin at operating range is known, so this doesn't reject
    // legitimate weak-but-real signals at the edge of range.
    constexpr int RSSI_FLOOR_DBM = -120;

    constexpr size_t MAX_PLAUSIBLE_WAYPOINTS = 64;
    // Placeholder bounds -- no real Jetson position units/scale specified
    // yet. Tighten once that's known.
    constexpr int16_t POSITION_BOUND = 20000;

    inline bool isPlausiblePosition(int16_t x, int16_t y){
        return x >= -POSITION_BOUND && x <= POSITION_BOUND && y >= -POSITION_BOUND && y <= POSITION_BOUND;
    }

    struct Waypoint{ int16_t x; int16_t y; };

    // Base -> rover response payload, reused verbatim as the rover -> Jetson
    // wire format too (no translation needed, see two_way_rover_main.cpp):
    //   [count:1][x:2][y:2]*count
    // count == 0xFF is a sentinel meaning "bad/incomplete data, rescan," not
    // a real waypoint count (real counts are capped at
    // MAX_PLAUSIBLE_WAYPOINTS, nowhere near 0xFF/255) -- the same sentinel
    // value is forwarded to Jetson unchanged.
    constexpr uint8_t BAD_DATA_SENTINEL = 0xFF;
    // Jetson-facing only (never travels over LoRa): rover's own give-up,
    // distinct from base explicitly reporting bad data.
    constexpr uint8_t GAVE_UP_SENTINEL = 0x00;

    inline std::vector<uint8_t> encodeGoodResponse(const std::vector<Waypoint>& waypoints){
        std::vector<uint8_t> bytes;
        bytes.reserve(1 + waypoints.size() * 4);
        bytes.push_back(static_cast<uint8_t>(waypoints.size()));
        for(const Waypoint& w : waypoints){
            bytes.push_back(static_cast<uint8_t>(w.x & 0xFF));
            bytes.push_back(static_cast<uint8_t>((w.x >> 8) & 0xFF));
            bytes.push_back(static_cast<uint8_t>(w.y & 0xFF));
            bytes.push_back(static_cast<uint8_t>((w.y >> 8) & 0xFF));
        }
        return bytes;
    }

    inline std::vector<uint8_t> encodeBadResponse(){
        return { BAD_DATA_SENTINEL };
    }

    // Returns false if bytes is malformed (wrong length for its own count
    // byte, or an implausible count). badData is set true for the 0xFF
    // sentinel (waypoints left empty); otherwise waypoints holds the
    // decoded list.
    inline bool decodeResponse(const std::vector<uint8_t>& bytes, bool& badData, std::vector<Waypoint>& waypoints){
        if(bytes.empty()) return false;
        uint8_t count = bytes[0];
        if(count == BAD_DATA_SENTINEL){
            badData = true;
            return true;
        }
        badData = false;
        if(count > MAX_PLAUSIBLE_WAYPOINTS) return false;
        if(bytes.size() != 1 + static_cast<size_t>(count) * 4) return false;
        for(size_t i = 0; i < count; i++){
            size_t off = 1 + i * 4;
            int16_t x = static_cast<int16_t>(bytes[off] | (bytes[off + 1] << 8));
            int16_t y = static_cast<int16_t>(bytes[off + 2] | (bytes[off + 3] << 8));
            waypoints.push_back(Waypoint{x, y});
        }
        return true;
    }

    // Position: direct bit manipulation, no compression -- 2 int16, 4 bytes,
    // little-endian (same convention as the Waypoint encoding above). A
    // handful of bytes has nothing for RLE to exploit, same reasoning
    // main.cpp already applies to waypoints.
    inline std::vector<uint8_t> encodePosition(int16_t x, int16_t y){
        return {
            static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((x >> 8) & 0xFF),
            static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF)
        };
    }
    inline bool decodePosition(const std::vector<uint8_t>& bytes, int16_t& x, int16_t& y){
        if(bytes.size() != 4) return false;
        x = static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
        y = static_cast<int16_t>(bytes[2] | (bytes[3] << 8));
        return true;
    }
}
#endif
