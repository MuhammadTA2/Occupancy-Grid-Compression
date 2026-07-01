#include "rle.h"
#include <iostream>
#include <random>
#include<vector>

using namespace compressor;
int main() {
	Grid grid_tx=createGrid(100,100);

	// Make sample occupancy grid
    	for (int i = 0; i < 100; i++) {
        	grid_tx.data[0 * grid_tx.cols + i] = 1;
        	grid_tx.data[99 * grid_tx.cols + i] = 1;
        	grid_tx.data[i * grid_tx.cols + 0] = 1;
        	grid_tx.data[i * grid_tx.cols + 99] = 1;
    }

    	for (int r = 10; r < 90; r++) {
        	grid_tx.data[r * grid_tx.cols + 20] = 1;
        	grid_tx.data[r * grid_tx.cols + 50] = 1;
        	grid_tx.data[r * grid_tx.cols + 75] = 1;
    }

    	for (int r = 30; r < 40; r++) grid_tx.data[r * grid_tx.cols + 20] = 0;
    	for (int r = 60; r < 70; r++) grid_tx.data[r * grid_tx.cols + 50] = 0;
    	for (int r = 20; r < 30; r++) grid_tx.data[r * grid_tx.cols + 75] = 0;

	std::vector<RLEbits> encoded= rleEncode(grid_tx.data);
	std::vector<int> decoded= rleDecode(encoded);
	Grid grid_rx= rebuildGrid(decoded,grid_tx.rows,grid_tx.cols);
			

	


	
	
	double cRatio=compressionRatio(grid_tx.data,encoded);

 	std::cout << "Original cells: " << grid_tx.data.size() << "\n";
    	std::cout << "Encoded RLE runs: " << encoded.size() << "\n";
    	std::cout << "Decoded cells: " << decoded.size() << "\n";
    	std::cout << "Rebuilt grid matches original: " << (grid_tx.data==grid_rx.data ? "YES" : "NO") << "\n";
    	std::cout << "Compression ratio: " << cRatio<< "\n";

    // Tile test
    	std::vector<Tile> tiles = splitGrid(grid_tx);
    	Grid tile_rebuilt = rebuildTiledGrid(tiles, grid_tx.rows, grid_tx.cols);

    	std::cout << "Tile count: " << tiles.size() << "\n";
    	std::cout << "Tile rebuild matches original: " << ((tile_rebuilt.data == grid_tx.data) ? "YES" : "NO") << "\n";
	return 0;

}
