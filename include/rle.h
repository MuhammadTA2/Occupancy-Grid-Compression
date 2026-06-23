#ifndef RLE_H
#define RLE_H
#include <iostream>
#include <vector>
namespace compressor{
	struct RLEbits {
		int value;
		int count;
	};
	struct Tile{
		int rowStart;
		int rowEnd;
		int row;
		int col;
		int data[row][col];
	};
	struct Grid{
		int row
		int col;;
		int data[row][col];
	};
	std::vector<Tile> splitGrid(co]);
	std::vector<int> flattenTile(const Tile& tile);
	std::vector<RLEbits> rleEncode(const std::vector<int>& data);
	std::vector<int> rleDecode(const std::vector<RLEbits>& code);
	Grid rebuildGrid(const std::vecotr<Tile>& tiles); 







}

#endif
