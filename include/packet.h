#ifndef PACKET_H
#define PACKET_H
// PROTOTYPE -- not wired into CMakeLists.txt, not part of any milestone yet.
// Written ahead of schedule; couples directly to RLEbits instead of an
// IEntropyCoder's byte output, and bundles LoRa-specific chunking into the
// same design. Kept as a design reference only -- see Discussion.md's
// 07/20/2026 entry. Milestone 6 will rebuild Packet against real
// IEntropyCoder bytes once Milestones 3-5 exist.
#include <vector>
#include <cstdint>
#include <cstddef>
#include "rle.h"
namespace compressor{

	// Wire format for one RLE run: [value:1 byte][count:2 bytes, little-endian] = 3 bytes/run.
	// This is NOT sizeof(RLEbits) -- struct padding in memory is not a transmission format,
	// and relying on it would silently change payload size (and break cross-compiler
	// compatibility) if the struct layout ever shifts.
	const size_t kRLERecordBytes = 3;

	std::vector<uint8_t> serializeRLE(const std::vector<RLEbits>& runs);
	std::vector<RLEbits> deserializeRLE(const std::vector<uint8_t>& bytes);

	uint8_t xorChecksum(const std::vector<uint8_t>& data);

	// --- Packetization ---
	// One tile's serialized RLE stream can be several KB; a single LoRa transmission
	// (SX127x explicit-header mode) tops out around 255 bytes. packetizeTile splits
	// a tile's payload into chunks that fit, each tagged with enough header info to
	// reassemble on the receiving ESP32.
	//
	// kDefaultMaxPayload leaves headroom under the 255-byte radio limit for this
	// module's own 6-byte header plus room for forward error correction bytes later
	// (FEC is listed as future work in ProjectSpecs.md and isn't implemented here).
	const uint8_t kDefaultMaxPayload = 200;

	#pragma pack(push, 1)
	struct PacketHeader {
		uint8_t tileId;       // which tile this packet belongs to (0..N-1 from splitGrid)
		uint8_t seq;          // this packet's index within the tile, starting at 0
		uint8_t totalSeqs;    // how many packets the tile was split into
		uint16_t payloadLen;  // number of valid bytes in this packet's payload
		uint8_t checksum;     // XOR checksum of payload bytes (lightweight, app-level;
		                      // complements rather than replaces the LoRa radio's own
		                      // optional packet CRC)
	};
	#pragma pack(pop)
	// NOTE: PacketHeader is sent as a packed struct assuming both ESP32s share the
	// same architecture/endianness (true for two Xtensa ESP32 boards). If this ever
	// talks to a different architecture, switch the header to explicit field-by-field
	// serialization the way serializeRLE/deserializeRLE already do for the payload.

	struct Packet {
		PacketHeader header;
		std::vector<uint8_t> payload;
	};

	std::vector<Packet> packetizeTile(uint8_t tileId, const std::vector<uint8_t>& serializedRLE,
	                                    uint8_t maxPayload = kDefaultMaxPayload);

	// Concatenates packet payloads in the order given. Caller is responsible for
	// collecting all totalSeqs packets for a tile and sorting them by seq first --
	// LoRa gives no ordering or delivery guarantee on its own.
	std::vector<uint8_t> reassembleTile(const std::vector<Packet>& packets);

}
#endif
