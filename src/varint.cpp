#include "varint.h"

namespace compressor{
	namespace varint{
		std::vector<uint8_t> encode(uint32_t value){
			std::vector<uint8_t> out;
			do{
				uint8_t byte = static_cast<uint8_t>(value & 0x7Fu);
				value >>= 7;
				if(value != 0) byte |= 0x80u;
				out.push_back(byte);
			} while(value != 0);
			return out;
		}

		bool decode(const std::vector<uint8_t>& data, size_t& pos, uint32_t& outValue){
			uint32_t result = 0;
			int shift = 0;
			size_t cursor = pos;

			while(true){
				if(cursor >= data.size()) return false; // truncated: stream ended mid-value
				if(shift >= 32) return false; // overflow: more continuation bytes than uint32_t can hold

				uint8_t byte = data[cursor];
				uint32_t chunk = byte & 0x7Fu;

				if(shift == 28 && (chunk & 0xF0u) != 0){
					// the 5th byte's low nibble already carries bits 28-31;
					// anything above that nibble can't fit in uint32_t
					return false;
				}

				result |= (chunk << shift);
				cursor++;
				shift += 7;

				if((byte & 0x80u) == 0) break; // continuation bit clear -- this was the last byte
			}

			pos = cursor;
			outValue = result;
			return true;
		}
	}
}
