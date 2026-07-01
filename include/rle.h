#ifndef RLE_H
#define RLE_H
#include <vector>
namespace compressor{
	const int tileSize=10;
	struct RLEbits {
		int value;
		int count;
	};
	struct Tile{
		int rowStart;
		int colStart;
		int rows;
		int cols;
		std::vector<int> data;
	};
	struct Grid{
		int rows;
		int cols;
		std::vector<int> data;
	};

	Grid createGrid(int rows, int cols);
	std::vector<Tile> splitGrid(const Grid& grid);
	Grid updateTiles(Grid grid, const std::vector<Tile>& changedTiles);
	//void setup

	std::vector<RLEbits> rleEncode(const std::vector<int>& data);
	//base station

	std::vector<int> rleDecode(const std::vector<RLEbits>& code);
	Grid rebuildTiledGrid(const std::vector<Tile>& tiles, int rows, int cols);
	Grid rebuildGrid(const std::vector<int>& flat, int rows, int cols);
	
	double compressionRatio(const std::vector<int>& uncompressed, const std::vector<RLEbits>& compressed);





}

#endif
