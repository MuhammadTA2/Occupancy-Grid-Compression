#include "rle.h"
//grid bro
namespace compressor{
	std::vector<Tile> splitGrid(const Grid& grid){
		std::vector<Tile> tiles; //define the tiles object to be returned, should contain 100 10x10 matrices
		for(int tileRow=0; tileRow<10; tileRow++){ 
			//keep track of which row we are in
			for( int tileCol=0; tileCol<10; tileCol++){
				//keep track of which collumn we are in
				Tile tile; //initialize tile object which will contain one 10x10 matrix
				tile.rowStart= tileRow*10; //tell tile where we are starting from
				tile.colStart=tileCol*10;
			
	
				for(int i=0; i<10; i++){
				
					for(int j=0; j<10; j++){
				
						tile.data[i][j]=grid.data[tile.rowStart+i][tile.colStart+j];
				}
				}
			tiles.push_back(tile);
			}
		}
			return tiles;
		}


	
	std::vector<int> flattenTile(const Tile& tile){
		std::vector<int> flat;
		for(int i=0;i<10;i++){
			for(int j=0; j<10;j++){
				flat.push_back(tile.data[i][j]);
			}
		}
		return flat;
	}


	std::vector<RLEbits> rleEncode(const std::vector<int>& data){
		//time to encode
		std::vector<RLEbits> code;
		//create vector to store compressed data
		if (data.empty()) return code;
		//create current that tells us what the current value is
		int current =data.front();
		//create RLEbits object that will get pushed to vector then reset on failed condition
		RLEbits temp;
		//loop will iterate through data, conditions will be if current==data[i] code.count++
		//when false, current = data[i], count gets stored
		int count=1;
		for( size_t i=1; i<data.size();i++){
			if(current!=data[i]){
				temp.count=count;
				temp.value=current;
				code.push_back(temp);
				current=data[i];
				count=1;
				continue;
			}
			count++;

			
		}
		RLEbits finalTemp;
		finalTemp.value=current;
		finalTemp.count=count;

		code.push_back(finalTemp);

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
	std::vector<Tile> reTile(const std::vector<int> flat){
		//Tile the flattneted (supposedly decoded) dataset so it can be rebuilt into a grid
		//create each tile to fill a row up to 10 collumns, and then start next row
		//start by creating a tile vector
		std::vector<Tile> tiles;

		for(int tileRow=0; tileRow<10; tileRow++){ 
			//keep track of which row we are in
			for( int tileCol=0; tileCol<10; tileCol++){
				//keep track of which collumn we are in
				Tile tile; //initialize tile object which will contain one 10x10 matrix
				tile.rowStart= tileRow*10; //tell tile where we are starting from
				tile.colStart=tileCol*10;
				for(int i=0; i<10; i++){
					for(int j=0; j<10;j++){
					
					tile.data[i][j]=flat[(tile.rowStart+i)*100+(tile.colStart+j)];
				
					}
				}
			tiles.push_back(tile);
			}
		}
		return tiles;
	
	}

	Grid rebuildGrid(const std::vector<Tile>& tiles){
		//put together the tiles
		Grid grid;
		
		grid.row=100;
		grid.col=100;
		
		for(size_t t=0;t<tiles.size();t++){
			const Tile& tile=tiles[t];
		
			for(int i=0; i<10;i++){
				for(int j=0; j<10; j++){
					grid.data[tile.rowStart+i][tile.colStart+j]=tile.data[i][j];	
				}
			}
			

		}
		return grid;
	}
	Grid updateTiles(Grid oldGrid, const std::vector<Tile>& changedTiles){
		if(changedTiles.empty()) return oldGrid;
		Grid updatedGrid= oldGrid;
		for(size_t t=0; t<changedTiles.size();t++){
			Tile tile= changedTiles[t];
			for(int r=0; r<10; r++){
				for(int c=0; c<10; c++){
					updatedGrid.data[tile.rowStart+r][tile.colStart+c]=tile.data[r][c];
				}
			}

		}
		return updatedGrid;
	}
}
