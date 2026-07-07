#include "rle.h"
namespace compressor{


	

	std::vector<RLEbits> rleEncode(const std::vector<int>& data){
	
		std::vector<RLEbits> code;
		//create vector to store compressed data
		if (data.empty()) return code;
		//create current that tells us what the current value is
		int current =data.front();

		//loop will iterate through data, conditions will be if current==data[i] code.count++
		//when false, current = data[i], count gets stored
		int count=1;
		for( size_t i=1; i<data.size();i++){
			if(current!=data[i]){
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

	

	std::vector<int> rleDecode(const std::vector<RLEbits>& code){
		//take in the vector of encoded data, then append value "count" times to create our flattened tile
		//initialize flattened vector after checking for empty code
		if(code.empty()) return {};
		std::vector<int> decoded;

		//loop for size of data, this should be nested, first is to access element, second is to append to vector
		for(size_t i=0; i<code.size(); i++){
			for(int j=0; j<code[i].count;j++){
				decoded.push_back(code[i].value);
			}
		}
		return decoded;
	}
	

}
