#include "counts_coder.h"
#include "varint_coder.h"
#include "rice_coder.h"

namespace compressor{
	namespace countscoder{
		std::vector<uint8_t> encode(CoderType type, const std::vector<uint16_t>& counts, uint8_t riceParam){
			switch(type){
				case CoderType::Varint: return varintcoder::encode(counts);
				case CoderType::Rice: return rice::encode(counts, riceParam);
				default: return {};
			}
		}

		bool decode(CoderType type, const std::vector<uint8_t>& data, size_t count, uint8_t riceParam, std::vector<uint16_t>& out){
			switch(type){
				case CoderType::Varint: return varintcoder::decode(data, count, out);
				case CoderType::Rice: return rice::decode(data, riceParam, count, out);
				default: return false;
			}
		}

		BestChoice chooseBest(const std::vector<uint16_t>& counts){
			BestChoice best;
			best.type = CoderType::Varint;
			best.riceParam = 0;
			best.bytes = varintcoder::encode(counts);

			for(uint8_t k = 0; k <= 12; k++){
				std::vector<uint8_t> candidate = rice::encode(counts, k);
				if(candidate.size() < best.bytes.size()){
					best.type = CoderType::Rice;
					best.riceParam = k;
					best.bytes = std::move(candidate);
				}
			}
			return best;
		}
	}
}
