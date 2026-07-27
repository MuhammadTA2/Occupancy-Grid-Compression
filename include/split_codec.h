#ifndef SPLIT_CODEC_H
#define SPLIT_CODEC_H

#include <cstdint>
#include <vector>
#include "rle.h"
#include "coder_type.h"

namespace compressor{
	// Unlike rlecodec (one combined self-contained blob), this produces two
	// independently-packetizable byte streams -- the shape Milestone 7's
	// packetizer actually expects: "the values stream and counts stream
	// belong to one logical message but are fragmented separately," for
	// fault containment against ordinary transport noise (a corrupted
	// values fragment shouldn't take down the counts stream too).
	//
	//   valuesBytes: FixedWidthCoder-packed values, no header at all -- its
	//                length is deterministic (ceil(N * valueBitWidth / 8))
	//                once N is known, and N travels in countsBytes.
	//   countsBytes: [CoderType: 1 byte][riceParam: 1 byte][N: varint][counts,
	//                encoded via whichever coder the header names]
	//
	// Both streams need N; putting it only in countsBytes (rather than
	// duplicating it in both) means decode() must read countsBytes first to
	// learn N before it can even slice valuesBytes -- that's fine, since the
	// caller (the receiver logic) has both streams in hand already once
	// PacketReassembler reports both complete.
	namespace splitcodec{
		struct EncodedStreams{
			std::vector<uint8_t> valuesBytes;
			std::vector<uint8_t> countsBytes;
		};

		// Automatically picks whichever counts coder (Varint or a
		// k-searched Rice) produces the smallest countsBytes for this
		// specific payload -- same auto-selection rlecodec::encode does.
		// Returns empty streams if runs.values.size() != runs.counts.size(),
		// or if any value doesn't fit in valueBitWidth bits.
		EncodedStreams encode(const RLERuns& runs, uint8_t valueBitWidth);

		// Explicit-choice overload, for callers (e.g. tests, or the
		// benchmark) that want to force a specific counts coder instead of
		// auto-selecting. Same rejection rules as the auto-select overload.
		EncodedStreams encode(const RLERuns& runs, uint8_t valueBitWidth, CoderType countsCoderType, uint8_t riceParam);

		// Returns false (leaving out unspecified) on a malformed counts
		// header, a truncated counts section, or a valuesBytes length that
		// doesn't match what countsBytes' own N implies.
		bool decode(const std::vector<uint8_t>& valuesBytes, const std::vector<uint8_t>& countsBytes,
		            uint8_t valueBitWidth, RLERuns& out);
	}
}
#endif
