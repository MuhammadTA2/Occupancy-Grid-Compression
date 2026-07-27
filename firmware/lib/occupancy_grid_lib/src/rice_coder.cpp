#include "rice_coder.h"
#include "bit_writer.h"

namespace compressor{
	namespace rice{
		std::vector<uint8_t> encode(const std::vector<uint16_t>& counts, uint8_t k){
			// A shift by >= the operand's bit width (32 for uint32_t) is
			// undefined behavior in C++, not just "wrong" -- reject before
			// value >> k below can ever hit that, rather than relying on
			// callers never passing a bad k.
			if(k >= 32) return {};

			BitWriter writer;
			uint32_t mask = (1u << k) - 1u;
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
			// k arrives from wire data (rlecodec/splitcodec's riceParam byte)
			// completely unvalidated -- reject k >= 32 here rather than let
			// quotient << k below hit undefined behavior on a corrupted or
			// adversarial packet.
			if(k >= 32) return false;

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
