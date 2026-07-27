#ifndef BIT_WRITER_H
#define BIT_WRITER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace compressor{
	// General-purpose bit-packing utility -- no knowledge of occupancy, RLE,
	// or varint. Appends values of a given bit width into a byte buffer,
	// MSB-first within each byte, packing values back-to-back across byte
	// boundaries. Used by FixedWidthCoder now; reusable by any future
	// bit-level entropy coder (Rice/Golomb, Huffman).
	class BitWriter{
	public:
		// Appends the low bitWidth bits of value to the stream.
		void writeBits(uint32_t value, uint8_t bitWidth);

		// Flushes any partially-filled trailing byte (zero-padded in the low
		// bits) and returns the packed buffer.
		std::vector<uint8_t> finish();

	private:
		std::vector<uint8_t> bytes_;
		uint8_t currentByte_ = 0;
		uint8_t bitsUsedInCurrentByte_ = 0;
	};

	class BitReader{
	public:
		explicit BitReader(const std::vector<uint8_t>& data);

		// Reads the next bitWidth bits into outValue and returns true, or
		// returns false if the stream doesn't have enough bits left
		// (truncated/malformed input) -- never guesses.
		bool readBits(uint8_t bitWidth, uint32_t& outValue);

		size_t bitsRemaining() const;

	private:
		const std::vector<uint8_t>& data_;
		size_t bitPos_ = 0;
	};
}
#endif
