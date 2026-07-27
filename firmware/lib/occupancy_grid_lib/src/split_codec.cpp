#include "split_codec.h"
#include "fixed_width_coder.h"
#include "counts_coder.h"
#include "varint.h"

namespace compressor{
	namespace splitcodec{
		EncodedStreams encode(const RLERuns& runs, uint8_t valueBitWidth, CoderType countsCoderType, uint8_t riceParam){
			EncodedStreams streams;
			if(runs.values.size() != runs.counts.size()) return streams;
			if(countsCoderType != CoderType::Varint && countsCoderType != CoderType::Rice) return streams;

			std::vector<uint8_t> valuesBytes;
			if(!fixedwidth::encode(runs.values, valueBitWidth, valuesBytes)) return EncodedStreams{};

			std::vector<uint8_t> countsPayload = countscoder::encode(countsCoderType, runs.counts, riceParam);

			std::vector<uint8_t> countsBytes;
			countsBytes.push_back(static_cast<uint8_t>(countsCoderType));
			countsBytes.push_back(riceParam);
			std::vector<uint8_t> nEncoded = varint::encode(static_cast<uint32_t>(runs.values.size()));
			countsBytes.insert(countsBytes.end(), nEncoded.begin(), nEncoded.end());
			countsBytes.insert(countsBytes.end(), countsPayload.begin(), countsPayload.end());

			streams.valuesBytes = std::move(valuesBytes);
			streams.countsBytes = std::move(countsBytes);
			return streams;
		}

		EncodedStreams encode(const RLERuns& runs, uint8_t valueBitWidth){
			if(runs.values.size() != runs.counts.size()) return EncodedStreams{};

			countscoder::BestChoice best = countscoder::chooseBest(runs.counts);
			return encode(runs, valueBitWidth, best.type, best.riceParam);
		}

		bool decode(const std::vector<uint8_t>& valuesBytes, const std::vector<uint8_t>& countsBytes,
		            uint8_t valueBitWidth, RLERuns& out){
			if(countsBytes.size() < 2) return false; // truncated: missing CoderType/riceParam header

			CoderType countsCoderType = static_cast<CoderType>(countsBytes[0]);
			uint8_t riceParam = countsBytes[1];
			if(countsCoderType != CoderType::Varint && countsCoderType != CoderType::Rice) return false;

			size_t pos = 2;
			uint32_t n = 0;
			if(!varint::decode(countsBytes, pos, n)) return false; // truncated/malformed N header

			size_t expectedValuesLen = (static_cast<size_t>(n) * valueBitWidth + 7) / 8;
			if(valuesBytes.size() != expectedValuesLen) return false; // the two streams disagree on N

			std::vector<uint8_t> values;
			if(!fixedwidth::decode(valuesBytes, valueBitWidth, n, values)) return false;

			std::vector<uint8_t> countsPayload(countsBytes.begin() + static_cast<long>(pos), countsBytes.end());
			std::vector<uint16_t> counts;
			if(!countscoder::decode(countsCoderType, countsPayload, n, riceParam, counts)) return false;

			out.values = std::move(values);
			out.counts = std::move(counts);
			return true;
		}
	}
}
