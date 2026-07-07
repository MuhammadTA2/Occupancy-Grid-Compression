#ifndef GRID_H
#define GRID_H
#include <vector>
namespace compressor{
	const int tileSize=10;
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

	Grid rebuildTiledGrid(const std::vector<Tile>& tiles, int rows, int cols);
	Grid rebuildGrid(const std::vector<int>& flat, int rows, int cols);


}
#endif
