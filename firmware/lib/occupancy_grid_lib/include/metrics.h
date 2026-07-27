#ifndef METRICS_H
#define METRICS_H
#include <cstddef>
namespace compressor{
	double compressionRatio(const size_t& compressed, const size_t& uncompressed);
}
#endif
