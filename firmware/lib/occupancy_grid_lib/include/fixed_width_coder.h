#ifndef FIXED_WIDTH_CODER_H
#define FIXED_WIDTH_CODER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace compressor{
	namespace fixedwidth{
		// Packs `values` into bytes, each value taking `bitWidth` bits
		// (MSB-first within each byte, via BitWriter). bitWidth is a runtime
		// parameter, not hardcoded, so other bounded-alphabet layers (future
		// map types) can reuse this at a different width than occupancy's
		// default of 2 bits (0=free, 1=obstacle, 2=uncertain, 11 reserved).
		//
		// Returns false (leaving out unspecified) if any value doesn't fit in
		// bitWidth bits -- rejected outright rather than silently truncated.
		bool encode(const std::vector<uint8_t>& values, uint8_t bitWidth, std::vector<uint8_t>& out);

		// Unpacks exactly `count` values of `bitWidth` bits each from `data`.
		// Returns false if data doesn't contain enough bits for `count`
		// values -- truncated/malformed input, never guessed at.
		bool decode(const std::vector<uint8_t>& data, uint8_t bitWidth, size_t count, std::vector<uint8_t>& out);
	}
}
#endif
