#ifndef SYMBOLTYPE_H
#define SYMBOLTYPE_H

#include <cstdint>

namespace compressor{
	// Structural tag for a SymbolStream's byte layout -- independent of entropy
	// coding. RLE means "structurally run-length encoded"; the actual coder
	// used to serialize values/counts is a separate concern (Milestone 4).
	enum class StreamFormat : uint8_t{
		Raw,
		RLE,
		Unknown  // sentinel for malformed/unrecognized streams
	};
}
#endif
