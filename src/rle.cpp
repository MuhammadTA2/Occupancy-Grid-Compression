#include "rle.h"
//grid bro
namespace compressor{
	Grid createGrid(int rows, int cols){
		Grid grid;
		grid.rows=rows;
		grid.cols=cols;
		grid.data.resize(rows*cols,0);
		return grid;
	}
	std::vector<Tile> splitGrid(const Grid& grid){
		std::vector<Tile> tiles; //define the tiles object to be returned, should contain 100 10x10 matrices
		for(int tileRow=0; tileRow<grid.rows; tileRow+=tileSize){ 
			//keep track of which row we are in
			for( int tileCol=0; tileCol<grid.cols; tileCol+=tileSize){
				//keep track of which collumn we are in
				Tile tile; //initialize tile object which will contain one 10x10 matrix
				tile.rowStart= tileRow; //tell tile where we are starting from
				tile.colStart=tileCol;
				tile.rows=std::min(tileSize,grid.rows-tile.rowStart);
				tile.cols=std::min(tileSize,grid.cols-tile.colStart);
			
	
				for(int i=0; i<tile.rows; i++){
				
					for(int j=0; j<tile.cols; j++){
				
						tile.data.push_back(grid.data[(tile.rowStart+i)*grid.cols+(tile.colStart+j)]);
				}
				}
			tiles.push_back(tile);
			}
		}
			return tiles;
		}


	

	std::vector<RLEbits> rleEncode(const std::vector<int>& data){
		//time to encode
		std::vector<RLEbits> code;
		//create vector to store compressed data
		if (data.empty()) return code;
		//create current that tells us what the current value is
		int current =data.front();

		//loop will iterate through data, conditions will be if current==data[i] code.count++
		//when false, current = data[i], count gets stored
		int count=1;
		for( size_t i=1; i<data.size();i++){
			if(current!=data[i]){
				code.push_back({current,count});
				current=data[i];
				count=1;
				continue;
			}
			count++;

			
		}

		code.push_back({current,count});

		return code;
		
		}

	

	std::vector<int> rleDecode(const std::vector<RLEbits>& code){
		//take in the vector of encoded data, then append value "count" times to create our flattened tile
		//initialize flattened vector after checking for empty code
		if(code.empty()) return {};
		std::vector<int> decoded;

		//loop for size of data, this should be nested, first is to access element, second is to append to vector
		for(size_t i=0; i<code.size(); i++){
			for(int j=0; j<code[i].count;j++){
				decoded.push_back(code[i].value);
			}
		}
		return decoded;
	}
	

	Grid rebuildTiledGrid(const std::vector<Tile>& tiles, int rows, int cols){
		//put together the tiles
		Grid grid =createGrid(rows,cols);
		
		for(size_t t=0;t<tiles.size();t++){
			const Tile& tile=tiles[t];
		
			for(int i=0; i<tile.rows;i++){
				for(int j=0; j<tile.cols; j++){
					grid.data[(tile.rowStart+i)*grid.cols+(tile.colStart+j)]=tile.data[i*tile.cols+j];	
				}
			}
			

		}
		return grid;
	}

	Grid rebuildGrid(const std::vector<int>& flat, int rows, int cols){
		Grid grid;
		grid.cols= cols;
		grid.rows=rows;
		grid.data=flat;
		return grid;
	}

	Grid updateTiles(Grid oldGrid, const std::vector<Tile>& changedTiles){
		if(changedTiles.empty()) return oldGrid;
		Grid updatedGrid= oldGrid;
		for(size_t t=0; t<changedTiles.size();t++){
			const Tile& tile= changedTiles[t];
			
			for(int r=0; r<tile.rows; r++){
				for(int c=0; c<tile.cols; c++){
					updatedGrid.data[(tile.rowStart+r)*updatedGrid.cols+(tile.colStart+c)]=tile.data[r*tile.cols+c];
				}
			}

		}
		return updatedGrid;

	}

	double compressionRatio(const std::vector<int>& uncompressed, const std::vector<RLEbits>& compressed){
    		if (compressed.empty()) return 0.0;

    		double originalSize = uncompressed.size() * sizeof(int);
    		double compressedSize = compressed.size() * sizeof(RLEbits);

    		return originalSize / compressedSize;
}
}
