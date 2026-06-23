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
				tile.rowStart= tilerow*10; //tell tile where we are starting from
				tile.colStart=tileCol*10;
				tile.rows=10;//tell tile how big the tile should be
				tile.cols=10;
			
	
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
		//create current that tells us what the current value is
		int current =data.front();
		//create RLEbits object that will get pushed to vector then reset on failed condition
		RLEbits temp;
		//loop will iterate through data, conditions will be if current==data[i] code.count++
		//when false, current = data[i], count gets stored
		int count=0;
		for( int i=0; i<data.size;i++){
			if(current!=data[i]){
				temp.count=count;
				temp.value=data[i-1];
				code.push_back(temp);
				current=data[i];
				count=1;
				continue;
			}
			count++;

			
		}
		
		}

	}

	std::vector<int> rleDecode(const std::vector<RLEbits>& code){

	}

	Grid rebuildGrid(const std::vector<Tile>& tiles){

	}
}
