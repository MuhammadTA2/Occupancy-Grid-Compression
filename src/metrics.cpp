#include "metrics.h"
namespace compressor{
	//potentially implement toBytes method using symbolStream to serialize dynamic types

	double compressionRatio(const size_t& compressed, const size_t& original){
    		if (compressed==0||original==0) return 0.0;
		
    		return static_cast<double>(original) / static_cast<double>(compressed);
}
}
