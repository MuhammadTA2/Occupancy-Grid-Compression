#ifndef RLE_H
#define RLE_H
#include <vector>
namespace compressor{
	struct RLEbits {
		int value;
		int count;
	};

	//void setup

	std::vector<RLEbits> rleEncode(const std::vector<int>& data);
	//base station

	std::vector<int> rleDecode(const std::vector<RLEbits>& code);
	





}

#endif
