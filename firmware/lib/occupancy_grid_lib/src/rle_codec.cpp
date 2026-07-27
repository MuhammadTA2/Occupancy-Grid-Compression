#include "rle_codec.h"
#include "fixed_width_coder.h"
#include "counts_coder.h"
#include "varint.h"

namespace compressor{
	namespace rlecodec{
		std::vector<uint8_t> encode(const RLERuns& runs, uint8_t valueBitWidth, CoderType countsCoderType, uint8_t riceParam){
			if(runs.values.size() != runs.counts.size()) return {};
			if(countsCoderType != CoderType::Varint && countsCoderType != CoderType::Rice) return {}; // unrecognized coder
			if(countsCoderType == CoderType::Rice && riceParam >= 32) return {}; // rice::encode would reject this anyway -- fail here, not with a silently-broken blob

			std::vector<uint8_t> packedValues;
			if(!fixedwidth::encode(runs.values, valueBitWidth, packedValues)) return {};

			std::vector<uint8_t> packedCounts = countscoder::encode(countsCoderType, runs.counts, riceParam);

			uint32_t n = static_cast<uint32_t>(runs.values.size());
			std::vector<uint8_t> out;
			out.push_back(static_cast<uint8_t>(countsCoderType));
			out.push_back(riceParam);

			std::vector<uint8_t> nEncoded = varint::encode(n);
			out.insert(out.end(), nEncoded.begin(), nEncoded.end());
			out.insert(out.end(), packedValues.begin(), packedValues.end());
			out.insert(out.end(), packedCounts.begin(), packedCounts.end());
			return out;
		}

		std::vector<uint8_t> encode(const RLERuns& runs, uint8_t valueBitWidth){
			if(runs.values.size() != runs.counts.size()) return {};

			countscoder::BestChoice best = countscoder::chooseBest(runs.counts);
			return encode(runs, valueBitWidth, best.type, best.riceParam);
		}

		bool decode(const std::vector<uint8_t>& data, uint8_t valueBitWidth, RLERuns& out){
			if(data.size() < 2) return false; // truncated: missing CoderType/riceParam header

			CoderType countsCoderType = static_cast<CoderType>(data[0]);
			uint8_t riceParam = data[1];
			if(countsCoderType != CoderType::Varint && countsCoderType != CoderType::Rice) return false;

			size_t pos = 2;
			uint32_t n = 0;
			if(!varint::decode(data, pos, n)) return false; // truncated/malformed run-count header

			size_t valuesByteLen = (static_cast<size_t>(n) * valueBitWidth + 7) / 8;
			if(pos + valuesByteLen > data.size()) return false; // truncated values section

			std::vector<uint8_t> valuesSection(data.begin() + static_cast<long>(pos),
			                                    data.begin() + static_cast<long>(pos + valuesByteLen));
			pos += valuesByteLen;

			std::vector<uint8_t> values;
			if(!fixedwidth::decode(valuesSection, valueBitWidth, n, values)) return false;

			std::vector<uint8_t> countsSection(data.begin() + static_cast<long>(pos), data.end());
			std::vector<uint16_t> counts;
			if(!countscoder::decode(countsCoderType, countsSection, n, riceParam, counts)) return false;

			out.values = std::move(values);
			out.counts = std::move(counts);
			return true;
		}
	}
}
