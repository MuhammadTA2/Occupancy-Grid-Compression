#include "rle.h"
namespace compressor{



	std::vector<RLEbits> rleEncode(const std::vector<uint8_t>& data){

		std::vector<RLEbits> code;
		//create vector to store compressed data
		if (data.empty()) return code;
		//create current that tells us what the current value is
		uint8_t current =data.front();

		//loop will iterate through data, conditions will be if current==data[i] code.count++
		//when false, current = data[i], count gets stored
		uint16_t count=1;
		for( size_t i=1; i<data.size();i++){
			// count is uint16_t (max 65535). Force a new run before it wraps to 0 --
			// only matters if a single grid ever grows past ~65535 identical cells
			// in a row, but silent wraparound would corrupt data, so guard it anyway.
			if(current!=data[i] || count==65535){
				code.push_back({current,count});
				current=data[i];
				count=1;
				continue;
			}
			count++;


		}

		code.push_back({current,count});

		return code;

		}



	std::vector<uint8_t> rleDecode(const std::vector<RLEbits>& code){
		//take in the vector of encoded data, then append value "count" times to create our flattened tile
		//initialize flattened vector after checking for empty code
		if(code.empty()) return {};
		std::vector<uint8_t> decoded;

		//loop for size of data, this should be nested, first is to access element, second is to append to vector
		for(size_t i=0; i<code.size(); i++){
			for(uint16_t j=0; j<code[i].count;j++){
				decoded.push_back(code[i].value);
			}
		}
		return decoded;
	}


}
