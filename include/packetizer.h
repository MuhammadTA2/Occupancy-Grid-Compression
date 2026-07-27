#ifndef PACKETIZER_H
#define PACKETIZER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace compressor{
	// Fragments an opaque byte stream into transport-sized packets and
	// serializes them to/from wire bytes. Zero semantic awareness of
	// compression -- it fragments/reassembles by raw byte position only.
	// streamId is an opaque caller-assigned tag (e.g. "this is the values
	// stream" vs "this is the counts stream" for one RLE message), not
	// hardcoded to any particular meaning here. The run count N produced by
	// rlecodec deliberately stays inside the payload, not a header field --
	// giving the packetizer compression-semantic knowledge would violate
	// this separation.
	namespace packetizer{
		// 255 is an SX127x/LoRa-derived default, not a universal constant --
		// revisit once real hardware (Milestone 12) reveals actual usable
		// payload limits.
		constexpr size_t MAX_PACKET_SIZE = 255;

		// version(1) + messageId(2) + streamId(1) + fragmentIndex(2) +
		// totalFragments(2) + payloadLength(1), each field fixed-width and
		// explicitly serialized (not sizeof(PacketHeader), which could
		// include compiler-dependent padding).
		constexpr size_t HEADER_SIZE = 9;

		constexpr size_t CRC_SIZE = 2; // CRC-16 over header+payload

		constexpr size_t MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - HEADER_SIZE - CRC_SIZE;

		struct PacketHeader{
			uint8_t version = 1;
			uint16_t messageId = 0;
			uint8_t streamId = 0;
			uint16_t fragmentIndex = 0;
			uint16_t totalFragments = 0;
			uint8_t payloadLength = 0;
		};

		struct Packet{
			PacketHeader header;
			std::vector<uint8_t> payload;
		};

		// CRC-16-CCITT (polynomial 0x1021, initial value 0xFFFF).
		uint16_t crc16(const std::vector<uint8_t>& data);

		// Splits data into packets of at most MAX_PAYLOAD_SIZE bytes each.
		// Empty data still produces exactly one packet (payloadLength=0,
		// totalFragments=1) so "this stream exists but is empty" and "this
		// stream doesn't exist" stay distinguishable downstream.
		std::vector<Packet> fragment(uint16_t messageId, uint8_t streamId, const std::vector<uint8_t>& data);

		// Serializes a Packet to its exact wire bytes: HEADER_SIZE +
		// payload.size() + CRC_SIZE (not padded to MAX_PACKET_SIZE).
		std::vector<uint8_t> serialize(const Packet& packet);

		// Parses wire bytes back into a Packet. Returns false (leaving out
		// unspecified) if the data is too short, payloadLength doesn't match
		// the actual remaining data length, or the CRC doesn't match --
		// rejected as corrupted/malformed, never guessed at.
		bool deserialize(const std::vector<uint8_t>& data, Packet& out);
	}
}
#endif
