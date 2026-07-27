#include "rle.h"

namespace compressor{

	RLERuns rleEncode(const std::vector<uint8_t>& data){
		RLERuns runs;
		if(data.empty()) return runs;

		size_t i = 0;
		while(i < data.size()){
			uint8_t value = data[i];
			uint16_t count = 0;
			// Force a new run before count would wrap past 65535 -- silent
			// wraparound would corrupt data.
			while(i < data.size() && data[i] == value && count < 65535){
				count++;
				i++;
			}
			runs.values.push_back(value);
			runs.counts.push_back(count);
		}
		return runs;
	}

	std::vector<uint8_t> rleDecode(const RLERuns& runs){
		if(runs.values.size() != runs.counts.size()) return {};
		std::vector<uint8_t> decoded;
		for(size_t i = 0; i < runs.values.size(); i++){
			decoded.insert(decoded.end(), runs.counts[i], runs.values[i]);
		}
		return decoded;
	}

}
