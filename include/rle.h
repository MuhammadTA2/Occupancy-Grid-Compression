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
	std::vector<int> flattenTile(const Tile& tile);

	std::vector<RLEbits> rleEncode(const std::vector<int>& data);
	std::vector<int> rleDecode(const std::vector<RLEbits>& code);
	
	std::vector<Tile> reTile(const std::vector<int>& flat);

	Grid rebuildGrid(const std::vector<Tile>& tiles);
	Grid rebuildGrid(const std::vector<int>& flat);
	Grid updateTiles(Grid grid, const std::vector<Tile>& changedTiles);






}

#endif
