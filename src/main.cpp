#include "rle.h"
#include <iostream>
#include <random>
#include<vector>

using namespace compressor;
int main() {
	Grid grid_tx;
	// Fill everything with free space
	for (int i = 0; i < 100; i++) {
    	for (int j = 0; j < 100; j++) {
        	grid_tx.data[i][j] = 0;
    }
}

// Outer walls
	for (int i = 0; i < 100; i++) {
    		grid_tx.data[0][i] = 1;
    		grid_tx.data[99][i] = 1;
   		grid_tx.data[i][0] = 1;
    		grid_tx.data[i][99] = 1;
}

// Vertical walls
	for (int r = 10; r < 90; r++) {
    		grid_tx.data[r][20] = 1;
    		grid_tx.data[r][50] = 1;
    		grid_tx.data[r][75] = 1;
}

// Doorways
	for (int r = 30; r < 40; r++){
    		grid_tx.data[r][20] = 0;
    }
	for (int r = 60; r < 70; r++){
    		grid_tx.data[r][50] = 0;
}	
	for (int r = 20; r < 30; r++){
    		grid_tx.data[r][75] = 0;
    }
// Horizontal walls
	for (int c = 15; c < 85; c++) {
    		grid_tx.data[25][c] = 1;
    		grid_tx.data[55][c] = 1;
 		grid_tx.data[80][c] = 1;
}

// Doorways
	for (int c = 40; c < 50; c++){
    		grid_tx.data[25][c] = 0;
    }
	for (int c = 60; c < 70; c++){
    		grid_tx.data[55][c] = 0;
}
	for (int c = 25; c < 35; c++){
    		grid_tx.data[80][c] = 0;
    }
// Obstacles
	for (int r = 12; r < 18; r++){
    		for (int c = 5; c < 12; c++){
        		grid_tx.data[r][c] = 1;
	}
}
	for (int r = 35; r < 45; r++){
    		for (int c = 30; c < 40; c++){
        		grid_tx.data[r][c] = 1;
	}
}
	for (int r = 62; r < 72; r++){
    		for (int c = 58; c < 68; c++){
        		grid_tx.data[r][c] = 1;
	}
}
	for (int r = 85; r < 95; r++){
    		for (int c = 82; c < 96; c++){
        		grid_tx.data[r][c] = 1;
	}
}

	std::vector<Tile> tiles= splitGrid(grid_tx);

	std::vector<int> flatGrid_tx=flattenGrid(grid_tx);

	

	std::vector<RLEbits> encoded= rleEncode(flatGrid_tx);
	std::vector<int> decoded= rleDecode(encoded);

	std::vector<Tile> rebuiltTiles= reTile(decoded);
	Grid grid_rx= rebuildGrid(rebuiltTiles);
	
	double cRatio=compressionRatio(flatGrid_tx,encoded);

	bool same=true;
	for(int i=0; i<100; i++){
		for(int j=0; j<100;j++){
			if(grid_rx.data[i][j]!=grid_tx.data[i][j]){
			same=false;
			break;
			}
			}
		}
	std::cout << "Original flat size: " << flatGrid_tx.size() << "\n";
	std::cout << "Encoded RLE runs: " << encoded.size() << "\n";
	std::cout << "Decoded flat size: " << decoded.size() << "\n";
 	std::cout << "Rebuilt grid matches original: " << (same ? "YES" : "NO") << "\n";

 	double originalBytes = flatGrid_tx.size() * sizeof(int);
 	double compressedBytes = encoded.size() * sizeof(RLEbits);

 	std::cout << "Original bytes: " << originalBytes << "\n";
 	std::cout << "Compressed bytes: " << compressedBytes << "\n";
 	std::cout << "Compression ratio: " << cRatio << "\n";

 	return 0;

}
