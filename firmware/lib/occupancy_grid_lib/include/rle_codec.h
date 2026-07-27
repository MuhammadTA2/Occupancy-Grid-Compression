#ifndef RLE_CODEC_H
#define RLE_CODEC_H

#include <cstdint>
#include <vector>
#include "rle.h"
#include "coder_type.h"

namespace compressor{
	// Combines RLE's structural output (RLERuns) with the coders that
	// actually serialize it into one self-contained byte blob:
	//
	//   [counts CoderType: 1 byte][riceParam: 1 byte]
	//   [run count N: varint]
	//   [values: N * valueBitWidth-bit packed via FixedWidthCoder]
	//   [counts: N values encoded via whichever coder the header names]
	//
	// Values always go through FixedWidthCoder (occupancy's alphabet is
	// small and fixed, so there's nothing to gain swapping that coder yet).
	// Counts are dispatched through countscoder, which can be Varint or
	// Rice -- the CoderType/riceParam header is what lets decode() figure
	// out which one without being told out-of-band, since nothing else
	// (packet metadata, Milestone 7) carries that yet.
	//
	// The leading N is required because bit-packed values are no longer
	// self-delimiting by EOF the way byte-per-run RLE was -- decode needs to
	// know upfront how many values/counts to unpack, and N's own encoding
	// also fixes the values section's byte length deterministically
	// (ceil(N * valueBitWidth / 8)), so no delimiter is needed between the
	// values and counts sections either.
	namespace rlecodec{
		// Automatically picks whichever counts coder (Varint or a
		// k-searched Rice) produces the smallest encoding for this specific
		// payload -- letting the data decide per payload is the point of
		// having more than one interchangeable counts coder, not locking in
		// a single "default" winner.
		std::vector<uint8_t> encode(const RLERuns& runs, uint8_t valueBitWidth);

		// Explicit-choice overload, for callers that want to force a
		// specific counts coder instead of auto-selecting.
		// Both overloads return an empty vector if runs.values.size() !=
		// runs.counts.size(), or if any value doesn't fit in valueBitWidth
		// bits -- rejected, not silently corrupted.
		std::vector<uint8_t> encode(const RLERuns& runs, uint8_t valueBitWidth, CoderType countsCoderType, uint8_t riceParam);

		// Returns false (leaving out unspecified) on any malformed/truncated
		// input: bad header, an unrecognized CoderType, a truncated values
		// section, or a truncated/invalid counts section. The counts
		// CoderType is read back out of the blob's own header, not supplied
		// by the caller.
		bool decode(const std::vector<uint8_t>& data, uint8_t valueBitWidth, RLERuns& out);
	}
}
#endif
