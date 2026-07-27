#include "bit_writer.h"

namespace compressor{
	void BitWriter::writeBits(uint32_t value, uint8_t bitWidth){
		for(int i = bitWidth - 1; i >= 0; i--){
			uint8_t bit = static_cast<uint8_t>((value >> i) & 1u);
			currentByte_ = static_cast<uint8_t>((currentByte_ << 1) | bit);
			bitsUsedInCurrentByte_++;
			if(bitsUsedInCurrentByte_ == 8){
				bytes_.push_back(currentByte_);
				currentByte_ = 0;
				bitsUsedInCurrentByte_ = 0;
			}
		}
	}

	std::vector<uint8_t> BitWriter::finish(){
		if(bitsUsedInCurrentByte_ > 0){
			currentByte_ = static_cast<uint8_t>(currentByte_ << (8 - bitsUsedInCurrentByte_));
			bytes_.push_back(currentByte_);
			currentByte_ = 0;
			bitsUsedInCurrentByte_ = 0;
		}
		return bytes_;
	}

	BitReader::BitReader(const std::vector<uint8_t>& data) : data_(data) {}

	bool BitReader::readBits(uint8_t bitWidth, uint32_t& outValue){
		if(bitsRemaining() < static_cast<size_t>(bitWidth)) return false;
		uint32_t value = 0;
		for(uint8_t i = 0; i < bitWidth; i++){
			size_t byteIndex = bitPos_ / 8;
			uint8_t bitIndexInByte = static_cast<uint8_t>(7 - (bitPos_ % 8));
			uint8_t bit = static_cast<uint8_t>((data_[byteIndex] >> bitIndexInByte) & 1u);
			value = (value << 1) | bit;
			bitPos_++;
		}
		outValue = value;
		return true;
	}

	size_t BitReader::bitsRemaining() const{
		return data_.size() * 8 - bitPos_;
	}
}
