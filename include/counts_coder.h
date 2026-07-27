#ifndef COUNTS_CODER_H
#define COUNTS_CODER_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include "coder_type.h"

namespace compressor{
	// Tag-dispatch layer over the interchangeable coders that can serialize
	// a sequence of RLE run counts (Varint today, Rice as a real alternative
	// -- see Milestone 5's benchmark). This is what lets a caller pick
	// whichever coder fits a given payload instead of one being hardcoded as
	// "the" answer; CoderType is the tag that travels with the encoded bytes
	// so decode knows which case to dispatch to. Switch-based, no vtables,
	// matching the project's ESP32-friendly mechanism.
	namespace countscoder{
		// riceParam is only meaningful when type == CoderType::Rice; ignored
		// (and unused) otherwise. Returns an empty vector for any type this
		// dispatcher doesn't recognize.
		std::vector<uint8_t> encode(CoderType type, const std::vector<uint16_t>& counts, uint8_t riceParam = 0);

		// Returns false if type isn't recognized, or if the underlying
		// coder's own decode rejects the data (truncated/overflowing/malformed).
		bool decode(CoderType type, const std::vector<uint8_t>& data, size_t count, uint8_t riceParam, std::vector<uint16_t>& out);

		struct BestChoice{
			CoderType type = CoderType::Unknown;
			uint8_t riceParam = 0;
			std::vector<uint8_t> bytes;
		};

		// Tries every known coder (Varint, and a small search over Rice's k
		// parameter) and returns whichever produced the smallest encoding for
		// this specific payload. This -- not a fixed default -- is the
		// intended way to use a family of interchangeable coders: let the
		// data decide, per payload.
		BestChoice chooseBest(const std::vector<uint16_t>& counts);
	}
}
#endif
