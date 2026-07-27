#ifndef VARINT_CODER_H
#define VARINT_CODER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace compressor{
	namespace varintcoder{
		// Encodes a sequence of run counts as back-to-back LEB128 values.
		// compressor::varint stays a single-integer primitive; this is the
		// component that actually serializes RLE's counts sequence.
		std::vector<uint8_t> encode(const std::vector<uint16_t>& counts);

		// Decodes exactly `count` LEB128 values from data. Returns false
		// (leaving out unspecified) on a truncated/malformed stream, or if a
		// decoded value overflows uint16_t -- rejected, not misdecoded.
		bool decode(const std::vector<uint8_t>& data, size_t count, std::vector<uint16_t>& out);
	}
}
#endif
