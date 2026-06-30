#ifndef RLE_H
#define RLE_H
#include <vector>
namespace compressor{
	struct RLEbits {
		int value;
		int count;
	};
	struct Tile{
		int rowStart;
		int colStart;
		int data[10][10];
	};
	struct Grid{
		int row;
		int col;
		int data[100][100];
	};
	std::vector<Tile> splitGrid(const Grid& grid);
	//void loop
	Grid updateTiles(Grid grid, const std::vector<Tile>& changedTiles);
	std::vector<int> flattenTiles(const std::vector<Tile>& tiles);
	//void setup
	std::vector<int> flattenGrid(const Grid& grid);

	std::vector<RLEbits> rleEncode(const std::vector<int>& data);
	//base station
	std::vector<Tile> reTile(const std::vector<int>& flat);

	std::vector<int> rleDecode(const std::vector<RLEbits>& code);
	Grid rebuildGrid(const std::vector<Tile>& tiles);
	Grid rebuildGrid(const std::vector<int>& flat);
	
	double compressionRatio(const std::vector<int>& uncompressed, const std::vector<RLEbits>& compressed);





}

#endif
