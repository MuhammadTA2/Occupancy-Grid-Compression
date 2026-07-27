#ifndef VARINT_H
#define VARINT_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace compressor{
	namespace varint{
		// Encodes value as unsigned LEB128: 7 data bits per byte, the MSB of
		// each byte is a continuation bit (1 = more bytes follow, 0 = this is
		// the last byte), least-significant group first. Carries no
		// RLE-specific meaning -- usable by any future preprocessor or packet
		// field that wants compact integers.
		std::vector<uint8_t> encode(uint32_t value);

		// Decodes a single LEB128 value starting at data[pos]. On success,
		// advances pos past the consumed bytes and returns true. On malformed
		// input -- the stream ends before a continuation bit clears, or more
		// than 5 bytes are needed (would overflow uint32_t) -- returns false
		// and leaves pos untouched, rather than guessing.
		bool decode(const std::vector<uint8_t>& data, size_t& pos, uint32_t& outValue);
	}
}
#endif
