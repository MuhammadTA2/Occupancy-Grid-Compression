#include "grid.h"
#include "rle.h"
#include "rle_codec.h"
#include "fixed_width_coder.h"
#include "counts_coder.h"
#include "metrics.h"

#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace compressor;

namespace {
	const uint8_t VALUE_BIT_WIDTH = 2;

	Grid makeUniformGrid(int rows, int cols){
		return createGrid(rows, cols); // all zero -- one giant run
	}

	Grid makeSparseObstacleGrid(int rows, int cols){
		Grid grid = createGrid(rows, cols);
		for(int i = 0; i < cols; i++){
			grid.data[0 * grid.cols + i] = 1;
			grid.data[(rows - 1) * grid.cols + i] = 1;
		}
		for(int r = 0; r < rows; r++){
			grid.data[r * grid.cols + 0] = 1;
			grid.data[r * grid.cols + (cols - 1)] = 1;
		}
		for(int r = rows / 10; r < rows - rows / 10; r++){
			grid.data[r * grid.cols + cols / 5] = 1;
			grid.data[r * grid.cols + cols / 2] = 1;
		}
		return grid;
	}

	Grid makeRandomNoiseGrid(int rows, int cols){
		Grid grid = createGrid(rows, cols);
		std::mt19937 rng(42); // fixed seed for reproducible benchmark results
		std::uniform_int_distribution<int> dist(0, 2);
		for(size_t i = 0; i < grid.data.size(); i++){
			grid.data[i] = static_cast<Cell>(dist(rng));
		}
		return grid;
	}

	Grid makeWorstCaseCheckerboardGrid(int rows, int cols){
		Grid grid = createGrid(rows, cols);
		for(size_t i = 0; i < grid.data.size(); i++){
			grid.data[i] = static_cast<Cell>(i % 2); // alternates on the flat index -- see test_main.cpp's note
		}
		return grid;
	}

	void runBenchmark(const std::string& name, const Grid& grid){
		RLERuns runs = rleEncode(grid.data);

		size_t structuralBytes = runs.values.size() * sizeof(uint8_t) + runs.counts.size() * sizeof(uint16_t);

		std::vector<uint8_t> fixedWidthValues;
		fixedwidth::encode(runs.values, VALUE_BIT_WIDTH, fixedWidthValues);

		size_t varintCountsBytes = countscoder::encode(CoderType::Varint, runs.counts).size();
		countscoder::BestChoice best = countscoder::chooseBest(runs.counts);

		size_t varintTotalBytes = fixedWidthValues.size() + varintCountsBytes;
		size_t bestTotalBytes = fixedWidthValues.size() + best.bytes.size();

		double rawSize = static_cast<double>(grid.data.size() * sizeof(Cell));
		const char* bestName = best.type == CoderType::Rice ? "Rice" : "Varint";

		std::cout << "--- " << name << " (" << grid.rows << "x" << grid.cols << ") ---\n";
		std::cout << "  Raw cells:                " << grid.data.size() << "\n";
		std::cout << "  RLE run count:            " << runs.values.size() << "\n";
		std::cout << "  Structural (AoS) bytes:   " << structuralBytes
		           << "  (ratio " << compressionRatio(structuralBytes, static_cast<size_t>(rawSize)) << ")\n";
		std::cout << "  FixedWidth+Varint bytes:  " << varintTotalBytes
		           << "  (values=" << fixedWidthValues.size() << ", counts=" << varintCountsBytes
		           << ", ratio " << compressionRatio(varintTotalBytes, static_cast<size_t>(rawSize)) << ")\n";
		std::cout << "  countscoder::chooseBest:  " << bestTotalBytes
		           << "  (values=" << fixedWidthValues.size() << ", counts=" << best.bytes.size()
		           << ", picked=" << bestName;
		if(best.type == CoderType::Rice) std::cout << " k=" << static_cast<int>(best.riceParam);
		std::cout << ", ratio " << compressionRatio(bestTotalBytes, static_cast<size_t>(rawSize)) << ")\n\n";
	}
}

int main(){
	std::cout << "Milestone 5 benchmark: forced Varint vs countscoder::chooseBest (Varint or Rice, picked per payload)\n";
	std::cout << "(Values coding is FixedWidthCoder in both -- only the counts coder differs.)\n\n";

	runBenchmark("Uniform (single run)", makeUniformGrid(100, 100));
	runBenchmark("Sparse obstacles", makeSparseObstacleGrid(100, 100));
	runBenchmark("Random noise (uniform 0/1/2)", makeRandomNoiseGrid(100, 100));
	runBenchmark("Worst-case checkerboard", makeWorstCaseCheckerboardGrid(100, 100));

	return 0;
}
