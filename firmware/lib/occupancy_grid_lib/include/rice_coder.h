#ifndef RICE_CODER_H
#define RICE_CODER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace compressor{
	// Golomb-Rice coding for a sequence of counts: each value's low k bits
	// are stored literally, the remaining high bits as a unary quotient
	// (q ones followed by a terminating zero). Benchmark-only (Milestone 5)
	// -- exists to check whether LEB128/VarintCoder was actually the right
	// choice for RLE run counts, not to replace it as the shipped default.
	namespace rice{
		std::vector<uint8_t> encode(const std::vector<uint16_t>& counts, uint8_t k);

		// Returns false on a malformed/truncated stream: the bit reader runs
		// out before `count` codes are decoded, the unary prefix never
		// terminates, or a decoded value overflows uint16_t.
		bool decode(const std::vector<uint8_t>& data, uint8_t k, size_t count, std::vector<uint16_t>& out);
	}
}
#endif
