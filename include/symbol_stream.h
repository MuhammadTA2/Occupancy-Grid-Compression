#ifndef SYMBOLSTREAM_H
#define SYMBOLSTREAM_H

#include "symbol_type.h"
#include <cstdint>
#include <vector>

namespace compressor{
	// Generic byte carrier between preprocessing and entropy coding.
	// symbols is uint8_t, not Cell -- this type has no knowledge of occupancy
	// semantics, so future map layers (height, confidence, ...) can reuse it.
	struct SymbolStream{
		StreamFormat format = StreamFormat::Unknown;
		std::vector<uint8_t> symbols;
	};

	inline bool operator==(const SymbolStream& a, const SymbolStream& b){
		return a.format == b.format && a.symbols == b.symbols;
	}
}
#endif
