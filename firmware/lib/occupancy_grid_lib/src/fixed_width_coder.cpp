#include "fixed_width_coder.h"
#include "bit_writer.h"

namespace compressor{
	namespace fixedwidth{
		bool encode(const std::vector<uint8_t>& values, uint8_t bitWidth, std::vector<uint8_t>& out){
			uint32_t maxValue = (1u << bitWidth) - 1u;
			for(uint8_t v : values){
				if(v > maxValue) return false; // doesn't fit in bitWidth bits
			}

			BitWriter writer;
			for(uint8_t v : values){
				writer.writeBits(static_cast<uint32_t>(v), bitWidth);
			}
			out = writer.finish();
			return true;
		}

		bool decode(const std::vector<uint8_t>& data, uint8_t bitWidth, size_t count, std::vector<uint8_t>& out){
			BitReader reader(data);
			std::vector<uint8_t> result;
			result.reserve(count);

			for(size_t i = 0; i < count; i++){
				uint32_t value = 0;
				if(!reader.readBits(bitWidth, value)) return false; // truncated stream
				result.push_back(static_cast<uint8_t>(value));
			}

			out = std::move(result);
			return true;
		}
	}
}
