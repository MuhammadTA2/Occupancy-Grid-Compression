#ifndef GRID_H
#define GRID_H
#include <vector>
#include <cstdint>
namespace compressor{
	const int tileSize=10;

	// Occupancy values: 0 = free, 1 = obstacle, 2 = uncertain.
	// uint8_t is plenty of range and cuts grid/tile memory 4x vs int,
	// which matters once this runs on an ESP32 instead of a desktop.
	using Cell = uint8_t;

	struct Tile{
		int rowStart;
		int colStart;
		int rows;
		int cols;
		std::vector<Cell> data;
	};
	struct Grid{
		int rows;
		int cols;
		std::vector<Cell> data;
	};

	Grid createGrid(int rows, int cols);
	std::vector<Tile> splitGrid(const Grid& grid);
	Grid updateTiles(Grid grid, const std::vector<Tile>& changedTiles);

	Grid rebuildTiledGrid(const std::vector<Tile>& tiles, int rows, int cols);
	Grid rebuildGrid(const std::vector<Cell>& flat, int rows, int cols);


}
#endif
