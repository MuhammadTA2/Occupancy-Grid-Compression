#ifndef RLE_H
#define RLE_H

#include <cstdint>
#include <vector>

namespace compressor{
	// Purely structural run-length transform: a flat byte sequence becomes
	// two separate ordered sequences -- all run values, then all run counts
	// (Structure-of-Arrays, not interleaved (value,count) pairs). This keeps
	// FixedWidthCoder (bit-packs values) and VarintCoder (LEB128-encodes
	// counts) each in their own independent, natural mode. RLE itself does
	// no byte-level serialization -- that's the coders' job (Milestone 4).
	struct RLERuns{
		std::vector<uint8_t> values;
		std::vector<uint16_t> counts;
	};

	// count is uint16_t, not uint8_t, because a single run can span an entire
	// 100x100 grid (10000 cells) -- forces a new run before count would wrap
	// past 65535.
	RLERuns rleEncode(const std::vector<uint8_t>& data);

	// Rejects (returns an empty vector) if values.size() != counts.size(),
	// rather than guessing which sequence is right -- a mismatched pair is
	// malformed input, e.g. from independently-corrupted fragments.
	std::vector<uint8_t> rleDecode(const RLERuns& runs);
}
#endif
