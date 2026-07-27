#include "varint_coder.h"
#include "varint.h"

namespace compressor{
	namespace varintcoder{
		std::vector<uint8_t> encode(const std::vector<uint16_t>& counts){
			std::vector<uint8_t> out;
			for(uint16_t c : counts){
				std::vector<uint8_t> encoded = varint::encode(static_cast<uint32_t>(c));
				out.insert(out.end(), encoded.begin(), encoded.end());
			}
			return out;
		}

		bool decode(const std::vector<uint8_t>& data, size_t count, std::vector<uint16_t>& out){
			std::vector<uint16_t> result;
			result.reserve(count);
			size_t pos = 0;

			for(size_t i = 0; i < count; i++){
				uint32_t value = 0;
				if(!varint::decode(data, pos, value)) return false;
				if(value > 0xFFFFu) return false; // overflow for uint16_t counts
				result.push_back(static_cast<uint16_t>(value));
			}

			out = std::move(result);
			return true;
		}
	}
}
