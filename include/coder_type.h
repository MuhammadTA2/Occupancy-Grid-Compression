#ifndef CODER_TYPE_H
#define CODER_TYPE_H

#include <cstdint>

namespace compressor{
	// Which coder produced/consumes a given byte sequence -- independent of
	// StreamFormat (which describes how symbols are structured, not which
	// algorithm turned them into bytes). This is the actual dispatch tag,
	// not just a label: IPreprocessor (RLE) hands off a structural sequence,
	// and any of several interchangeable IEntropyCoder-role implementations
	// can be picked to serialize it, per payload, at encode time -- that's
	// the whole point of keeping preprocessing and entropy coding as
	// separate roles (see Discussion.md 07/06/2026). Whoever decodes a
	// stream needs to know which of these produced it before decoding can
	// even start; once the packetizer exists (Milestone 7) this belongs in
	// packet metadata, but until then rlecodec embeds it directly in its own
	// self-contained blob header.
	enum class CoderType : uint8_t{
		FixedWidth, // bit-packed fixed-width values (fixed_width_coder.h)
		Varint,     // LEB128-encoded integers (varint_coder.h)
		Rice,       // Golomb-Rice coded integers (rice_coder.h)
		Unknown     // sentinel for malformed/unrecognized streams
	};
}
#endif
