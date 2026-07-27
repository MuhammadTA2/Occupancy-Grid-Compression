#include "rice_coder.h"
#include "bit_writer.h"

namespace compressor{
	namespace rice{
		std::vector<uint8_t> encode(const std::vector<uint16_t>& counts, uint8_t k){
			BitWriter writer;
			uint32_t mask = (k >= 32) ? 0xFFFFFFFFu : ((1u << k) - 1u);
			for(uint16_t v : counts){
				uint32_t value = static_cast<uint32_t>(v);
				uint32_t quotient = value >> k;
				uint32_t remainder = value & mask;
				for(uint32_t i = 0; i < quotient; i++) writer.writeBits(1, 1);
				writer.writeBits(0, 1);
				if(k > 0) writer.writeBits(remainder, k);
			}
			return writer.finish();
		}

		bool decode(const std::vector<uint8_t>& data, uint8_t k, size_t count, std::vector<uint16_t>& out){
			BitReader reader(data);
			std::vector<uint16_t> result;
			result.reserve(count);

			for(size_t i = 0; i < count; i++){
				uint32_t quotient = 0;
				uint32_t bit = 0;
				while(true){
					if(!reader.readBits(1, bit)) return false; // truncated unary prefix
					if(bit == 0) break;
					quotient++;
					if(quotient > 100000u) return false; // runaway unary prefix -- malformed/adversarial
				}

				uint32_t remainder = 0;
				if(k > 0){
					if(!reader.readBits(k, remainder)) return false; // truncated remainder
				}

				uint32_t value = (quotient << k) | remainder;
				if(value > 0xFFFFu) return false; // overflow for uint16_t counts
				result.push_back(static_cast<uint16_t>(value));
			}

			out = std::move(result);
			return true;
		}
	}
}
