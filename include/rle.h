#ifndef RLE_H
#define RLE_H

#include <cstddef>
#include <vector>
#include <cstdint>
#include <cstddef>
namespace compressor{
	// A run of `count` consecutive cells of `value`.
	// value is uint8_t (matches Grid/Tile's Cell type: 0=free, 1=obstacle, 2=uncertain).
	// count is uint16_t, not uint8_t, because a single run can span an entire
	// 100x100 grid (10000 cells) -- that doesn't fit in 8 bits but fits easily in 16.
	struct RLEbits {
		uint8_t value;
		uint16_t count;
	};

	//void setup

	std::vector<RLEbits> rleEncode(const std::vector<uint8_t>& data);
	//base station

	std::vector<uint8_t> rleDecode(const std::vector<RLEbits>& code);






}

#endif
