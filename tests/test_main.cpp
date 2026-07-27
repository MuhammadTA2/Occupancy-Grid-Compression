#include "rle.h"
#include "grid.h"
#include "metrics.h"
#include "bit_writer.h"
#include "varint.h"
#include "fixed_width_coder.h"
#include "varint_coder.h"
#include "rice_coder.h"
#include "counts_coder.h"
#include "rle_codec.h"
#include "packetizer.h"
#include "loopback_transport.h"
#include "packet_reassembler.h"
#include "split_codec.h"
#include "lossy_transport.h"
#include "arq.h"

#include <algorithm>
#include <iostream>
#include <string>

using namespace compressor;

namespace {
	int failures = 0;

	void check(bool condition, const std::string& name) {
		if (condition) {
			std::cout << "[PASS] " << name << "\n";
		} else {
			std::cout << "[FAIL] " << name << "\n";
			failures++;
		}
	}

	template <typename T>
	void checkEqual(const T& actual, const T& expected, const std::string& name) {
		if (actual == expected) {
			std::cout << "[PASS] " << name << "\n";
		} else {
			std::cout << "[FAIL] " << name << " (expected " << expected << ", got " << actual << ")\n";
			failures++;
		}
	}
}

void test_createGrid() {
	Grid grid = createGrid(4, 5);
	checkEqual(grid.rows, 4, "createGrid: rows");
	checkEqual(grid.cols, 5, "createGrid: cols");
	checkEqual(grid.data.size(), static_cast<size_t>(20), "createGrid: data size is rows*cols");
	bool allZero = true;
	for (int v : grid.data) if (v != 0) allZero = false;
	check(allZero, "createGrid: data is zero-initialized");
}

void test_rleEncodeDecode_roundTrip() {
	SymbolStream data{StreamFormat::Raw, {1, 1, 1, 0, 0, 2, 2, 2, 2}};
	RLERuns encoded = rleEncode(data.symbols);
	std::vector<uint8_t> decodedRaw = rleDecode(encoded);
	SymbolStream decoded{StreamFormat::Raw, decodedRaw};
	checkEqual(encoded.values.size(), static_cast<size_t>(3), "rleEncode: run count for {1,1,1,0,0,2,2,2,2}");
	check(decoded == data, "rleEncode/rleDecode: round trip preserves data");
}

void test_rleEncode_singleRun() {
	std::vector<uint8_t> data = {5, 5, 5, 5};
	RLERuns encoded = rleEncode(data);
	bool ok = encoded.values.size() == 1 && encoded.counts.size() == 1 &&
	          encoded.values[0] == 5 && encoded.counts[0] == 4;
	check(ok, "rleEncode: single run collapses to one run {value=5,count=4}");
}

void test_rleEncodeDecode_empty() {
	std::vector<uint8_t> empty;
	RLERuns encoded = rleEncode(empty);
	std::vector<uint8_t> decoded = rleDecode(encoded);
	check(encoded.values.empty() && encoded.counts.empty(), "rleEncode: empty input produces empty values/counts");
	check(decoded.empty(), "rleDecode: empty input produces empty output");
}

void test_rleDecode_mismatchedLengthsRejected() {
	RLERuns malformed;
	malformed.values = {1, 2, 3};
	malformed.counts = {1, 1}; // one short -- values/counts must travel in lockstep
	std::vector<uint8_t> decoded = rleDecode(malformed);
	check(decoded.empty(), "rleDecode: mismatched values/counts lengths reject rather than misdecode");
}

void test_bitWriter_bitReader_roundTrip() {
	BitWriter writer;
	std::vector<uint8_t> input = {0, 1, 2, 3, 1, 0};
	for (uint8_t v : input) writer.writeBits(v, 2);
	std::vector<uint8_t> packed = writer.finish();

	BitReader reader(packed);
	std::vector<uint8_t> output;
	bool allOk = true;
	for (size_t i = 0; i < input.size(); i++) {
		uint32_t value = 0;
		if (!reader.readBits(2, value)) { allOk = false; break; }
		output.push_back(static_cast<uint8_t>(value));
	}
	check(allOk && output == input, "BitWriter/BitReader: round trip at 2-bit width");
}

void test_bitWriter_bitReader_nonByteAlignedTrailingBits() {
	// 3 values at 3 bits each = 9 bits, not a multiple of 8 -- exercises the
	// finish() flush/padding path.
	BitWriter writer;
	std::vector<uint8_t> input = {5, 0, 7};
	for (uint8_t v : input) writer.writeBits(v, 3);
	std::vector<uint8_t> packed = writer.finish();

	checkEqual(packed.size(), static_cast<size_t>(2), "BitWriter: 9 bits flush to 2 bytes (padded)");

	BitReader reader(packed);
	std::vector<uint8_t> output;
	bool allOk = true;
	for (size_t i = 0; i < input.size(); i++) {
		uint32_t value = 0;
		if (!reader.readBits(3, value)) { allOk = false; break; }
		output.push_back(static_cast<uint8_t>(value));
	}
	check(allOk && output == input, "BitWriter/BitReader: round trip on non-byte-aligned bit width");
}

void test_bitReader_truncatedStreamRejected() {
	std::vector<uint8_t> oneByte = {0xFF};
	BitReader reader(oneByte);
	uint32_t value = 0;
	check(reader.readBits(2, value), "BitReader: first 2-bit read succeeds within one byte");
	check(reader.readBits(2, value), "BitReader: second 2-bit read succeeds within one byte");
	check(reader.readBits(2, value), "BitReader: third 2-bit read succeeds within one byte");
	check(reader.readBits(2, value), "BitReader: fourth 2-bit read succeeds within one byte (8 bits exactly used)");
	check(!reader.readBits(2, value), "BitReader: fifth 2-bit read fails -- stream exhausted, rejected not guessed");
}

void test_varint_roundTrip_variousValues() {
	std::vector<uint32_t> samples = {0, 1, 5, 127, 128, 300, 16384, 4294967295u};
	bool allOk = true;
	for (uint32_t v : samples) {
		std::vector<uint8_t> encoded = varint::encode(v);
		size_t pos = 0;
		uint32_t decoded = 0;
		if (!varint::decode(encoded, pos, decoded) || decoded != v || pos != encoded.size()) {
			allOk = false;
			break;
		}
	}
	check(allOk, "varint: round trip on 0, small, multi-byte, and max uint32 values");
}

void test_varint_decode_truncatedStreamRejected() {
	std::vector<uint8_t> encoded = varint::encode(300); // needs 2 bytes (continuation bit set on the first)
	std::vector<uint8_t> truncated = {encoded[0]}; // drop the continuation byte
	size_t pos = 0;
	uint32_t decoded = 0;
	check(!varint::decode(truncated, pos, decoded), "varint::decode: truncated multi-byte stream is rejected");
}

void test_varint_decode_excessContinuationBytesRejected() {
	// 6 bytes, every one with the continuation bit set -- more bytes than a
	// uint32 value can ever need (max 5), so this must be rejected as
	// malformed rather than decoded into some huge/wrapped value.
	std::vector<uint8_t> malformed = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	size_t pos = 0;
	uint32_t decoded = 0;
	check(!varint::decode(malformed, pos, decoded), "varint::decode: excess continuation bytes are rejected");
}

void test_fixedWidthCoder_roundTrip() {
	std::vector<uint8_t> values = {0, 1, 2, 1, 0, 2, 2};
	std::vector<uint8_t> packed;
	bool encodeOk = fixedwidth::encode(values, 2, packed);

	std::vector<uint8_t> decoded;
	bool decodeOk = fixedwidth::decode(packed, 2, values.size(), decoded);

	check(encodeOk && decodeOk && decoded == values, "FixedWidthCoder: round trip at 2-bit width");
}

void test_fixedWidthCoder_rejectsOutOfRangeValue() {
	std::vector<uint8_t> values = {0, 1, 4}; // 4 doesn't fit in 2 bits (max 3)
	std::vector<uint8_t> packed;
	check(!fixedwidth::encode(values, 2, packed), "FixedWidthCoder::encode: rejects a value that doesn't fit bitWidth");
}

void test_fixedWidthCoder_decode_truncatedRejected() {
	std::vector<uint8_t> values = {1, 2};
	std::vector<uint8_t> packed;
	fixedwidth::encode(values, 2, packed); // 4 bits used, 1 byte total
	std::vector<uint8_t> decoded;
	check(!fixedwidth::decode(packed, 2, 5, decoded), "FixedWidthCoder::decode: asking for more values than the data holds is rejected");
}

void test_varintCoder_roundTrip() {
	std::vector<uint16_t> counts = {1, 300, 0, 65535, 4};
	std::vector<uint8_t> encoded = varintcoder::encode(counts);
	std::vector<uint16_t> decoded;
	bool ok = varintcoder::decode(encoded, counts.size(), decoded);
	check(ok && decoded == counts, "VarintCoder: round trip on a realistic counts sequence incl. 0 and uint16_t max");
}

void test_varintCoder_decode_overflowRejected() {
	// Encode a raw value that doesn't fit in uint16_t directly with varint,
	// then ask VarintCoder to decode it as one of its counts.
	std::vector<uint8_t> encoded = varint::encode(70000);
	std::vector<uint16_t> decoded;
	check(!varintcoder::decode(encoded, 1, decoded), "VarintCoder::decode: a value overflowing uint16_t is rejected");
}

void test_countsCoder_dispatchesVarint() {
	std::vector<uint16_t> counts = {1, 300, 0, 65535, 4};
	std::vector<uint8_t> viaDispatch = countscoder::encode(CoderType::Varint, counts);
	std::vector<uint8_t> direct = varintcoder::encode(counts);
	check(viaDispatch == direct, "countscoder::encode: CoderType::Varint dispatches to varintcoder");

	std::vector<uint16_t> decoded;
	bool ok = countscoder::decode(CoderType::Varint, viaDispatch, counts.size(), 0, decoded);
	check(ok && decoded == counts, "countscoder::decode: CoderType::Varint round trips");
}

void test_countsCoder_dispatchesRice() {
	std::vector<uint16_t> counts = {1, 1, 2, 1, 3, 1, 1};
	uint8_t k = 1;
	std::vector<uint8_t> viaDispatch = countscoder::encode(CoderType::Rice, counts, k);
	std::vector<uint8_t> direct = rice::encode(counts, k);
	check(viaDispatch == direct, "countscoder::encode: CoderType::Rice dispatches to rice with the given k");

	std::vector<uint16_t> decoded;
	bool ok = countscoder::decode(CoderType::Rice, viaDispatch, counts.size(), k, decoded);
	check(ok && decoded == counts, "countscoder::decode: CoderType::Rice round trips");
}

void test_rice_encode_rejectsShiftWidthOverflow() {
	std::vector<uint16_t> counts = {1, 2, 3};
	// k >= 32 would make value >> k undefined behavior (shift width for a
	// 32-bit operand) -- must be rejected, not silently produce garbage.
	check(rice::encode(counts, 32).empty(), "rice::encode: rejects k == 32 (shift-width overflow)");
	check(rice::encode(counts, 200).empty(), "rice::encode: rejects k == 200 (shift-width overflow)");
	check(!rice::encode(counts, 31).empty(), "rice::encode: k == 31 (just below the boundary) still works");
}

void test_rice_decode_rejectsShiftWidthOverflow() {
	std::vector<uint16_t> counts = {1, 2, 3};
	std::vector<uint8_t> encoded = rice::encode(counts, 4);
	std::vector<uint16_t> decoded;

	// Same encoded bytes, but decode is asked to use an out-of-range k --
	// this is exactly what a corrupted/adversarial riceParam byte on the
	// wire would produce. quotient << k must never execute for k >= 32.
	check(!rice::decode(encoded, 32, counts.size(), decoded), "rice::decode: rejects k == 32 (shift-width overflow)");
	check(!rice::decode(encoded, 255, counts.size(), decoded), "rice::decode: rejects k == 255 (shift-width overflow)");
}

void test_rlecodec_encode_rejectsRiceShiftWidthOverflow() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	check(rlecodec::encode(runs, 2, CoderType::Rice, 32).empty(),
	      "rlecodec::encode: rejects CoderType::Rice with riceParam == 32 rather than emitting a broken blob");
}

void test_splitcodec_encode_rejectsRiceShiftWidthOverflow() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	splitcodec::EncodedStreams streams = splitcodec::encode(runs, 2, CoderType::Rice, 32);
	check(streams.valuesBytes.empty() && streams.countsBytes.empty(),
	      "splitcodec::encode: rejects CoderType::Rice with riceParam == 32 rather than emitting a broken stream");
}

void test_rlecodec_decode_rejectsAdversarialRiceParam() {
	// Hand-craft a blob exactly like the one a corrupted/adversarial packet
	// would produce: a structurally valid header naming CoderType::Rice,
	// but with a riceParam byte that would be UB if it reached rice::decode
	// unchecked. encode() can no longer produce this itself (it now
	// rejects up front -- see the test above), so this simulates data
	// arriving over the wire some other way (bit flip, malicious sender).
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	std::vector<uint8_t> validBlob = rlecodec::encode(runs, 2, CoderType::Rice, 4);
	check(!validBlob.empty(), "test setup: a legitimate Rice-coded blob with riceParam=4 encodes fine");

	std::vector<uint8_t> corrupted = validBlob;
	corrupted[1] = 200; // stomp the riceParam byte with an out-of-range value

	RLERuns decodedRuns;
	check(!rlecodec::decode(corrupted, 2, decodedRuns),
	      "rlecodec::decode: rejects a corrupted riceParam byte (>= 32) instead of hitting UB in rice::decode");
}

void test_splitcodec_decode_rejectsAdversarialRiceParam() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	splitcodec::EncodedStreams streams = splitcodec::encode(runs, 2, CoderType::Rice, 4);
	check(!streams.countsBytes.empty(), "test setup: a legitimate Rice-coded stream with riceParam=4 encodes fine");

	std::vector<uint8_t> corruptedCounts = streams.countsBytes;
	corruptedCounts[1] = 255; // stomp the riceParam byte with an out-of-range value

	RLERuns decodedRuns;
	check(!splitcodec::decode(streams.valuesBytes, corruptedCounts, 2, decodedRuns),
	      "splitcodec::decode: rejects a corrupted riceParam byte (>= 32) instead of hitting UB in rice::decode");
}

void test_countsCoder_chooseBest_picksSmallerAndRoundTrips() {
	// Many small, repetitive counts -- exactly the shape where Rice's
	// per-value bit cost beats Varint's byte-granular floor.
	std::vector<uint16_t> counts;
	for (int i = 0; i < 200; i++) counts.push_back(static_cast<uint16_t>(1 + (i % 3)));

	countscoder::BestChoice best = countscoder::chooseBest(counts);
	std::vector<uint8_t> varintBytes = varintcoder::encode(counts);

	check(best.bytes.size() <= varintBytes.size(), "countscoder::chooseBest: never picks something worse than Varint");
	check(best.type == CoderType::Rice, "countscoder::chooseBest: picks Rice for small/repetitive counts");

	std::vector<uint16_t> decoded;
	bool ok = countscoder::decode(best.type, best.bytes, counts.size(), best.riceParam, decoded);
	check(ok && decoded == counts, "countscoder::chooseBest: the chosen coder's own output round trips via countscoder::decode");
}

void test_rleCodec_autoSelect_embedsWinningCoderType() {
	// Same repetitive-counts shape as above, run through the full rlecodec
	// blob -- confirms the CoderType header is actually written and read
	// back correctly, not just chosen and discarded.
	std::vector<uint8_t> data;
	for (int i = 0; i < 200; i++) {
		uint8_t run = static_cast<uint8_t>(1 + (i % 3));
		for (int j = 0; j < run; j++) data.push_back(static_cast<uint8_t>(i % 2));
	}
	RLERuns runs = rleEncode(data);
	std::vector<uint8_t> autoBlob = rlecodec::encode(runs, 2);
	std::vector<uint8_t> forcedVarintBlob = rlecodec::encode(runs, 2, CoderType::Varint, 0);

	check(autoBlob.size() <= forcedVarintBlob.size(), "rlecodec: auto-select is never larger than forcing Varint");
	checkEqual(static_cast<int>(autoBlob[0]), static_cast<int>(CoderType::Rice), "rlecodec: auto-select embeds Rice as the winning CoderType for this payload");

	RLERuns decodedRuns;
	bool ok = rlecodec::decode(autoBlob, 2, decodedRuns);
	std::vector<uint8_t> roundTripped = rleDecode(decodedRuns);
	check(ok && roundTripped == data, "rlecodec: auto-selected blob round trips correctly");
}

void test_rleCodec_decode_rejectsUnrecognizedCoderType() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	std::vector<uint8_t> blob = rlecodec::encode(runs, 2, CoderType::Varint, 0);
	blob[0] = 99; // stomp the CoderType byte with a value no enumerator maps to

	RLERuns decodedRuns;
	check(!rlecodec::decode(blob, 2, decodedRuns), "rlecodec::decode: an unrecognized CoderType byte is rejected");
}

void test_rleCodec_roundTrip() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	std::vector<uint8_t> blob = rlecodec::encode(runs, 2);

	RLERuns decodedRuns;
	bool ok = rlecodec::decode(blob, 2, decodedRuns);
	std::vector<uint8_t> roundTripped = rleDecode(decodedRuns);

	check(ok && decodedRuns.values == runs.values && decodedRuns.counts == runs.counts,
	      "rlecodec: encode/decode round trip preserves values and counts");
	check(roundTripped == data, "rlecodec: full round trip through RLE reproduces original data");
}

void test_rleCodec_rejectsMismatchedRuns() {
	RLERuns malformed;
	malformed.values = {1, 2, 3};
	malformed.counts = {1, 1};
	check(rlecodec::encode(malformed, 2).empty(), "rlecodec::encode: rejects mismatched values/counts lengths");
}

void test_rleCodec_rejectsValueOutOfBitWidth() {
	RLERuns runs;
	runs.values = {0, 1, 5}; // 5 doesn't fit in 2 bits
	runs.counts = {1, 1, 1};
	check(rlecodec::encode(runs, 2).empty(), "rlecodec::encode: rejects a value that doesn't fit valueBitWidth");
}

void test_rleCodec_decode_truncatedRejected() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	std::vector<uint8_t> blob = rlecodec::encode(runs, 2);
	std::vector<uint8_t> truncated(blob.begin(), blob.end() - 1); // drop the last byte

	RLERuns decodedRuns;
	check(!rlecodec::decode(truncated, 2, decodedRuns), "rlecodec::decode: truncated blob is rejected");
}

void test_fullPipeline_roundTrip_emptyGrid() {
	Grid grid = createGrid(0, 0);
	SymbolStream stream = toSymbolStream(grid);
	RLERuns runs = rleEncode(stream.symbols);
	std::vector<uint8_t> blob = rlecodec::encode(runs, 2);

	RLERuns decodedRuns;
	bool ok = rlecodec::decode(blob, 2, decodedRuns);
	std::vector<uint8_t> decodedSymbols = rleDecode(decodedRuns);
	Grid rebuilt = fromSymbolStream(SymbolStream{StreamFormat::Raw, decodedSymbols}, 0, 0);

	check(ok && rebuilt.data.empty(), "full pipeline: empty grid round trips through RLE+coders to an empty grid");
}

void test_fullPipeline_roundTrip_singleRun() {
	Grid grid = createGrid(5, 5); // all zero -- one giant run
	SymbolStream stream = toSymbolStream(grid);
	RLERuns runs = rleEncode(stream.symbols);
	std::vector<uint8_t> blob = rlecodec::encode(runs, 2);

	RLERuns decodedRuns;
	bool ok = rlecodec::decode(blob, 2, decodedRuns);
	std::vector<uint8_t> decodedSymbols = rleDecode(decodedRuns);
	Grid rebuilt = fromSymbolStream(SymbolStream{StreamFormat::Raw, decodedSymbols}, grid.rows, grid.cols);

	checkEqual(runs.values.size(), static_cast<size_t>(1), "full pipeline: uniform grid collapses to a single run");
	check(ok && rebuilt.data == grid.data, "full pipeline: single-run grid round trips correctly");
}

void test_fullPipeline_roundTrip_worstCaseCheckerboard() {
	// Alternate directly on the flattened index, not (row+col)%2 -- with an
	// even column count, (row+col)%2 repeats the same value across the
	// row-to-row seam (last cell of row r and first cell of row r+1 land on
	// the same parity), which merges what should be two 1-cell runs into one
	// 2-cell run at every seam. Flat-index alternation is what actually
	// guarantees every adjacent element in RLE's row-major input differs.
	Grid grid = createGrid(10, 10);
	for (size_t i = 0; i < grid.data.size(); i++) {
		grid.data[i] = static_cast<Cell>(i % 2);
	}
	SymbolStream stream = toSymbolStream(grid);
	RLERuns runs = rleEncode(stream.symbols);
	std::vector<uint8_t> blob = rlecodec::encode(runs, 2);

	RLERuns decodedRuns;
	bool ok = rlecodec::decode(blob, 2, decodedRuns);
	std::vector<uint8_t> decodedSymbols = rleDecode(decodedRuns);
	Grid rebuilt = fromSymbolStream(SymbolStream{StreamFormat::Raw, decodedSymbols}, grid.rows, grid.cols);

	checkEqual(runs.values.size(), grid.data.size(), "full pipeline: checkerboard forces maximum run count (one run per cell)");
	check(ok && rebuilt.data == grid.data, "full pipeline: worst-case checkerboard grid round trips correctly");
}

void test_packetizer_fragment_smallData() {
	std::vector<uint8_t> data = {1, 2, 3, 4, 5};
	std::vector<packetizer::Packet> packets = packetizer::fragment(42, 1, data);

	bool ok = packets.size() == 1 &&
	          packets[0].header.messageId == 42 &&
	          packets[0].header.streamId == 1 &&
	          packets[0].header.fragmentIndex == 0 &&
	          packets[0].header.totalFragments == 1 &&
	          packets[0].header.payloadLength == 5 &&
	          packets[0].payload == data;
	check(ok, "packetizer::fragment: data smaller than MAX_PAYLOAD_SIZE produces a single fragment");
}

void test_packetizer_fragment_emptyData() {
	std::vector<uint8_t> empty;
	std::vector<packetizer::Packet> packets = packetizer::fragment(1, 0, empty);

	bool ok = packets.size() == 1 &&
	          packets[0].header.totalFragments == 1 &&
	          packets[0].header.payloadLength == 0 &&
	          packets[0].payload.empty();
	check(ok, "packetizer::fragment: empty data still produces one empty-payload fragment (exists but empty)");
}

void test_packetizer_fragment_multipleFragments_reassembleByPosition() {
	std::vector<uint8_t> data;
	for(int i = 0; i < 500; i++) data.push_back(static_cast<uint8_t>(i % 256));

	std::vector<packetizer::Packet> packets = packetizer::fragment(7, 3, data);

	size_t expectedFragments = (data.size() + packetizer::MAX_PAYLOAD_SIZE - 1) / packetizer::MAX_PAYLOAD_SIZE;
	checkEqual(packets.size(), expectedFragments, "packetizer::fragment: 500 bytes splits into ceil(500/MAX_PAYLOAD_SIZE) fragments");

	std::vector<uint8_t> reassembled;
	bool indicesOk = true;
	for(size_t i = 0; i < packets.size(); i++){
		if(packets[i].header.fragmentIndex != i || packets[i].header.totalFragments != packets.size()) indicesOk = false;
		reassembled.insert(reassembled.end(), packets[i].payload.begin(), packets[i].payload.end());
	}
	check(indicesOk, "packetizer::fragment: fragmentIndex is sequential and totalFragments is consistent across all fragments");
	check(reassembled == data, "packetizer::fragment: concatenating fragment payloads in order reproduces the original data");
}

void test_packetizer_serialize_deserialize_roundTrip() {
	packetizer::Packet p;
	p.header.version = 1;
	p.header.messageId = 1234;
	p.header.streamId = 9;
	p.header.fragmentIndex = 2;
	p.header.totalFragments = 5;
	p.header.payloadLength = 4;
	p.payload = {10, 20, 30, 40};

	std::vector<uint8_t> bytes = packetizer::serialize(p);
	packetizer::Packet decoded;
	bool ok = packetizer::deserialize(bytes, decoded);

	bool fieldsMatch = ok &&
	                    decoded.header.version == p.header.version &&
	                    decoded.header.messageId == p.header.messageId &&
	                    decoded.header.streamId == p.header.streamId &&
	                    decoded.header.fragmentIndex == p.header.fragmentIndex &&
	                    decoded.header.totalFragments == p.header.totalFragments &&
	                    decoded.header.payloadLength == p.header.payloadLength &&
	                    decoded.payload == p.payload;
	check(fieldsMatch, "packetizer: serialize/deserialize round trip preserves every header field and the payload");
}

void test_packetizer_deserialize_rejectsTruncated() {
	packetizer::Packet p;
	p.header.messageId = 1;
	p.header.streamId = 0;
	p.header.totalFragments = 1;
	p.header.payloadLength = 3;
	p.payload = {1, 2, 3};
	std::vector<uint8_t> bytes = packetizer::serialize(p);
	std::vector<uint8_t> truncated(bytes.begin(), bytes.end() - 3); // chop off part of the payload+CRC

	packetizer::Packet decoded;
	check(!packetizer::deserialize(truncated, decoded), "packetizer::deserialize: truncated packet is rejected");
}

void test_packetizer_deserialize_rejectsCorruptedCrc() {
	packetizer::Packet p;
	p.header.messageId = 5;
	p.header.streamId = 2;
	p.header.totalFragments = 1;
	p.header.payloadLength = 4;
	p.payload = {9, 9, 9, 9};
	std::vector<uint8_t> bytes = packetizer::serialize(p);
	bytes[packetizer::HEADER_SIZE] ^= 0xFF; // flip a payload byte without touching the CRC

	packetizer::Packet decoded;
	check(!packetizer::deserialize(bytes, decoded), "packetizer::deserialize: a corrupted payload byte fails CRC and is rejected");
}

void test_packetizer_deserialize_rejectsPayloadLengthMismatch() {
	packetizer::Packet p;
	p.header.messageId = 7;
	p.header.streamId = 1;
	p.header.totalFragments = 1;
	p.header.payloadLength = 5; // claims 5 payload bytes...
	p.payload = {1, 2, 3};      // ...but only 3 are actually present/serialized

	std::vector<uint8_t> bytes = packetizer::serialize(p); // CRC is internally consistent with what's actually written
	packetizer::Packet decoded;
	check(!packetizer::deserialize(bytes, decoded),
	      "packetizer::deserialize: payloadLength inconsistent with actual data length is rejected even with a matching CRC");
}

void test_loopbackTransport_receiveEmptyReturnsFalse() {
	LoopbackTransport transport;
	std::vector<uint8_t> out;
	check(!transport.receive(out), "LoopbackTransport::receive: returns false when nothing has been sent");
}

void test_loopbackTransport_sendReceive_fifoOrder() {
	LoopbackTransport transport;
	std::vector<uint8_t> first = {1, 2, 3};
	std::vector<uint8_t> second = {4, 5};

	check(transport.send(first), "LoopbackTransport::send: accepts the first packet");
	check(transport.send(second), "LoopbackTransport::send: accepts the second packet");
	checkEqual(transport.pending(), static_cast<size_t>(2), "LoopbackTransport::pending: reports 2 queued packets");

	std::vector<uint8_t> receivedFirst, receivedSecond;
	bool gotFirst = transport.receive(receivedFirst);
	bool gotSecond = transport.receive(receivedSecond);

	check(gotFirst && gotSecond && receivedFirst == first && receivedSecond == second,
	      "LoopbackTransport: receive() returns packets in the same FIFO order they were sent");

	std::vector<uint8_t> out;
	check(!transport.receive(out), "LoopbackTransport::receive: returns false once the queue is drained");
}

void test_loopbackTransport_carriesPacketizerBytes() {
	// Sanity check that ITransport and packetizer compose the way the full
	// pipeline (Milestone 10) will actually use them: fragment -> serialize
	// -> send -> receive -> deserialize.
	std::vector<uint8_t> data = {10, 20, 30, 40, 50};
	std::vector<packetizer::Packet> packets = packetizer::fragment(1, 0, data);

	LoopbackTransport transport;
	for(const packetizer::Packet& p : packets) transport.send(packetizer::serialize(p));

	std::vector<uint8_t> reassembled;
	bool allOk = true;
	for(size_t i = 0; i < packets.size(); i++){
		std::vector<uint8_t> raw;
		if(!transport.receive(raw)) { allOk = false; break; }
		packetizer::Packet decoded;
		if(!packetizer::deserialize(raw, decoded)) { allOk = false; break; }
		reassembled.insert(reassembled.end(), decoded.payload.begin(), decoded.payload.end());
	}

	check(allOk && reassembled == data, "LoopbackTransport + packetizer: fragment/serialize/send/receive/deserialize round trips data");
}

void test_reassembler_singleFragment_completeImmediately() {
	std::vector<uint8_t> data = {1, 2, 3};
	std::vector<packetizer::Packet> packets = packetizer::fragment(1, 0, data);

	PacketReassembler reassembler;
	check(!reassembler.isComplete(1, 0), "PacketReassembler: unknown (messageId, streamId) is not complete");
	check(reassembler.receive(packets[0]), "PacketReassembler::receive: accepts a valid single fragment");
	check(reassembler.isComplete(1, 0), "PacketReassembler: complete after its one fragment arrives");

	std::vector<uint8_t> out;
	check(reassembler.tryGetCompleteStream(1, 0, out) && out == data,
	      "PacketReassembler::tryGetCompleteStream: reassembles a single-fragment stream correctly");
}

void test_reassembler_multipleFragments_outOfOrder_reassembleCorrectly() {
	std::vector<uint8_t> data;
	for(int i = 0; i < 500; i++) data.push_back(static_cast<uint8_t>(i % 256));
	std::vector<packetizer::Packet> packets = packetizer::fragment(2, 1, data);
	check(packets.size() >= 3, "test setup: 500 bytes produces at least 3 fragments to exercise reordering");

	PacketReassembler reassembler;
	// Feed fragments in reverse order -- reassembly must not depend on
	// arrival order, only on each fragment's own fragmentIndex.
	for(auto it = packets.rbegin(); it != packets.rend(); ++it){
		reassembler.receive(*it);
	}

	check(reassembler.isComplete(2, 1), "PacketReassembler: complete once all fragments have arrived, regardless of order");
	std::vector<uint8_t> out;
	check(reassembler.tryGetCompleteStream(2, 1, out) && out == data,
	      "PacketReassembler: reassembles byte-exact data even when fragments arrived out of order");
}

void test_reassembler_incomplete_returnsFalse() {
	std::vector<uint8_t> data;
	for(int i = 0; i < 500; i++) data.push_back(static_cast<uint8_t>(i));
	std::vector<packetizer::Packet> packets = packetizer::fragment(3, 0, data);
	check(packets.size() >= 2, "test setup: needs at least 2 fragments");

	PacketReassembler reassembler;
	reassembler.receive(packets[0]); // withhold the rest

	check(!reassembler.isComplete(3, 0), "PacketReassembler: not complete with fragments missing");
	std::vector<uint8_t> out;
	check(!reassembler.tryGetCompleteStream(3, 0, out), "PacketReassembler::tryGetCompleteStream: fails while incomplete");
}

void test_reassembler_duplicateFragment_idempotent() {
	std::vector<uint8_t> data = {7, 8, 9};
	std::vector<packetizer::Packet> packets = packetizer::fragment(4, 0, data);

	PacketReassembler reassembler;
	check(reassembler.receive(packets[0]), "PacketReassembler::receive: accepts the fragment the first time");
	check(reassembler.receive(packets[0]), "PacketReassembler::receive: accepts the same fragment again (idempotent, not an error)");
	check(reassembler.isComplete(4, 0), "PacketReassembler: a duplicated fragment doesn't prevent completion");

	std::vector<uint8_t> out;
	check(reassembler.tryGetCompleteStream(4, 0, out) && out == data,
	      "PacketReassembler: duplicate delivery doesn't corrupt the reassembled data");
}

void test_reassembler_inconsistentTotalFragments_rejected() {
	packetizer::Packet a;
	a.header.messageId = 5;
	a.header.streamId = 0;
	a.header.fragmentIndex = 0;
	a.header.totalFragments = 2;
	a.payload = {1};

	packetizer::Packet b = a;
	b.header.fragmentIndex = 1;
	b.header.totalFragments = 3; // disagrees with what fragment `a` already established

	PacketReassembler reassembler;
	check(reassembler.receive(a), "PacketReassembler::receive: accepts the first fragment establishing totalFragments=2");
	check(!reassembler.receive(b), "PacketReassembler::receive: rejects a later fragment disagreeing on totalFragments");
}

void test_reassembler_fragmentIndexOutOfRange_rejected() {
	packetizer::Packet p;
	p.header.messageId = 6;
	p.header.streamId = 0;
	p.header.fragmentIndex = 5;
	p.header.totalFragments = 3; // index 5 is out of range for only 3 total fragments
	p.payload = {1};

	PacketReassembler reassembler;
	check(!reassembler.receive(p), "PacketReassembler::receive: rejects a fragmentIndex >= totalFragments");
}

void test_reassembler_missingFragments_reportsCorrectIndices() {
	std::vector<uint8_t> data;
	for(int i = 0; i < 500; i++) data.push_back(static_cast<uint8_t>(i));
	std::vector<packetizer::Packet> packets = packetizer::fragment(7, 0, data);
	check(packets.size() >= 3, "test setup: needs at least 3 fragments");

	PacketReassembler reassembler;
	reassembler.receive(packets[0]);
	reassembler.receive(packets[2]); // skip index 1

	std::vector<uint16_t> missing = reassembler.missingFragments(7, 0);
	bool onlyIndexOneMissing = missing.size() == packets.size() - 2;
	for(size_t i = 0; i < packets.size(); i++){
		bool shouldBeMissing = (i != 0 && i != 2);
		bool isReportedMissing = std::find(missing.begin(), missing.end(), static_cast<uint16_t>(i)) != missing.end();
		if(shouldBeMissing != isReportedMissing) onlyIndexOneMissing = false;
	}
	check(onlyIndexOneMissing, "PacketReassembler::missingFragments: reports exactly the not-yet-received indices");
}

void test_reassembler_independentStreamsByKey() {
	std::vector<uint8_t> valuesData = {1, 1, 1};
	std::vector<uint8_t> countsData = {2, 2, 2};
	std::vector<packetizer::Packet> valuesPackets = packetizer::fragment(8, 0, valuesData); // same messageId,
	std::vector<packetizer::Packet> countsPackets = packetizer::fragment(8, 1, countsData); // different streamId

	PacketReassembler reassembler;
	reassembler.receive(valuesPackets[0]);
	check(reassembler.isComplete(8, 0) && !reassembler.isComplete(8, 1),
	      "PacketReassembler: (messageId, streamId=0) completing doesn't mark streamId=1 complete");

	reassembler.receive(countsPackets[0]);
	std::vector<uint8_t> outValues, outCounts;
	reassembler.tryGetCompleteStream(8, 0, outValues);
	reassembler.tryGetCompleteStream(8, 1, outCounts);
	check(outValues == valuesData && outCounts == countsData,
	      "PacketReassembler: values and counts streams under the same messageId stay independent by streamId");
}

void test_splitCodec_roundTrip() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	splitcodec::EncodedStreams streams = splitcodec::encode(runs, 2);

	check(!streams.valuesBytes.empty() && !streams.countsBytes.empty(), "splitcodec::encode: produces non-empty values and counts streams");

	RLERuns decodedRuns;
	bool ok = splitcodec::decode(streams.valuesBytes, streams.countsBytes, 2, decodedRuns);
	std::vector<uint8_t> roundTripped = rleDecode(decodedRuns);

	check(ok && decodedRuns.values == runs.values && decodedRuns.counts == runs.counts,
	      "splitcodec: decode recovers the original values and counts");
	check(roundTripped == data, "splitcodec: full round trip through RLE reproduces original data");
}

void test_splitCodec_explicitChoice_forcesRequestedCoder() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);

	splitcodec::EncodedStreams varintStreams = splitcodec::encode(runs, 2, CoderType::Varint, 0);
	splitcodec::EncodedStreams riceStreams = splitcodec::encode(runs, 2, CoderType::Rice, 3);

	checkEqual(static_cast<int>(varintStreams.countsBytes[0]), static_cast<int>(CoderType::Varint),
	           "splitcodec::encode (explicit): forcing Varint embeds CoderType::Varint in countsBytes");
	checkEqual(static_cast<int>(riceStreams.countsBytes[0]), static_cast<int>(CoderType::Rice),
	           "splitcodec::encode (explicit): forcing Rice embeds CoderType::Rice in countsBytes");
	checkEqual(static_cast<int>(riceStreams.countsBytes[1]), 3,
	           "splitcodec::encode (explicit): forcing Rice embeds the requested riceParam in countsBytes");

	RLERuns decodedFromVarint, decodedFromRice;
	bool varintOk = splitcodec::decode(varintStreams.valuesBytes, varintStreams.countsBytes, 2, decodedFromVarint);
	bool riceOk = splitcodec::decode(riceStreams.valuesBytes, riceStreams.countsBytes, 2, decodedFromRice);

	check(varintOk && decodedFromVarint.values == runs.values && decodedFromVarint.counts == runs.counts,
	      "splitcodec: forced-Varint streams round trip correctly");
	check(riceOk && decodedFromRice.values == runs.values && decodedFromRice.counts == runs.counts,
	      "splitcodec: forced-Rice streams round trip correctly");
}

void test_splitCodec_rejectsMismatchedRuns() {
	RLERuns malformed;
	malformed.values = {1, 2, 3};
	malformed.counts = {1, 1};
	splitcodec::EncodedStreams streams = splitcodec::encode(malformed, 2);
	check(streams.valuesBytes.empty() && streams.countsBytes.empty(), "splitcodec::encode: rejects mismatched values/counts lengths");
}

void test_splitCodec_decode_rejectsDisagreeingStreams() {
	std::vector<uint8_t> data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	RLERuns runs = rleEncode(data);
	splitcodec::EncodedStreams streams = splitcodec::encode(runs, 2);

	std::vector<uint8_t> tamperedValues = streams.valuesBytes;
	tamperedValues.push_back(0); // now longer than what countsBytes' own N implies

	RLERuns decodedRuns;
	check(!splitcodec::decode(tamperedValues, streams.countsBytes, 2, decodedRuns),
	      "splitcodec::decode: rejects when the values stream's length disagrees with the counts stream's N");
}

void test_lossyTransport_zeroDropProbability_neverDrops() {
	LossyTransport transport(0.0);
	for(int i = 0; i < 20; i++) transport.send({static_cast<uint8_t>(i)});
	checkEqual(transport.droppedCount(), static_cast<size_t>(0), "LossyTransport: dropProbability=0 drops nothing");
	checkEqual(transport.pending(), static_cast<size_t>(20), "LossyTransport: dropProbability=0 delivers every packet to the queue");
}

void test_lossyTransport_oneDropProbability_alwaysDrops() {
	LossyTransport transport(1.0);
	for(int i = 0; i < 20; i++) transport.send({static_cast<uint8_t>(i)});
	checkEqual(transport.droppedCount(), static_cast<size_t>(20), "LossyTransport: dropProbability=1 drops everything");
	checkEqual(transport.pending(), static_cast<size_t>(0), "LossyTransport: dropProbability=1 leaves the queue empty");
}

void test_lossyTransport_partialLoss_conservesCount() {
	LossyTransport transport(0.5);
	const int total = 100;
	for(int i = 0; i < total; i++) transport.send({static_cast<uint8_t>(i)});
	checkEqual(transport.pending() + transport.droppedCount(), static_cast<size_t>(total),
	           "LossyTransport: every sent packet is accounted for as either delivered or dropped");
}

void test_arq_drainInto_reliableTransport_deliversEverything() {
	std::vector<uint8_t> data;
	for(int i = 0; i < 500; i++) data.push_back(static_cast<uint8_t>(i));
	std::vector<packetizer::Packet> packets = packetizer::fragment(1, 0, data);

	LoopbackTransport transport;
	for(const auto& p : packets) transport.send(packetizer::serialize(p));

	PacketReassembler reassembler;
	arq::drainInto(transport, reassembler);

	check(reassembler.isComplete(1, 0), "arq::drainInto: a reliable transport delivers a complete stream in one drain");
}

void test_arq_sendWithRetry_recoversFromPartialLoss() {
	std::vector<uint8_t> data;
	for(int i = 0; i < 2000; i++) data.push_back(static_cast<uint8_t>(i % 251));

	LossyTransport transport(0.3); // fixed seed -- reproducible loss pattern
	PacketReassembler reassembler;

	arq::Result result = arq::sendWithRetry(transport, reassembler, 1, 0, data, 10);

	check(result.success, "arq::sendWithRetry: eventually succeeds over a lossy transport given enough retries");
	check(result.roundsUsed > 1, "arq::sendWithRetry: actually needed at least one retry round given 30% loss");

	std::vector<uint8_t> reassembled;
	check(reassembler.tryGetCompleteStream(1, 0, reassembled) && reassembled == data,
	      "arq::sendWithRetry: the eventually-reassembled data matches the original exactly");
}

void test_arq_sendWithRetry_givesUpAfterMaxRetries() {
	std::vector<uint8_t> data = {1, 2, 3, 4, 5};

	LossyTransport transport(1.0); // drops everything, unconditionally
	PacketReassembler reassembler;

	arq::Result result = arq::sendWithRetry(transport, reassembler, 2, 0, data, 3);

	check(!result.success, "arq::sendWithRetry: reports failure when the transport drops everything past the retry limit");
	checkEqual(result.roundsUsed, 4, "arq::sendWithRetry: stops after 1 initial round + 3 retries, doesn't loop forever");
}

void test_toSymbolStream_fromSymbolStream_roundTrip() {
	Grid grid = createGrid(3, 4);
	for (size_t i = 0; i < grid.data.size(); i++) {
		grid.data[i] = static_cast<Cell>(i % 3);
	}
	SymbolStream stream = toSymbolStream(grid);
	Grid rebuilt = fromSymbolStream(stream, grid.rows, grid.cols);
	check(rebuilt.data == grid.data, "toSymbolStream/fromSymbolStream: round trip preserves cell values");
}

void test_toSymbolStream_tagsRaw() {
	Grid grid = createGrid(2, 2);
	SymbolStream stream = toSymbolStream(grid);
	check(stream.format == StreamFormat::Raw, "toSymbolStream: tags output as StreamFormat::Raw");
}

void test_toSymbolStream_fromSymbolStream_emptyGrid() {
	Grid grid = createGrid(0, 0);
	SymbolStream stream = toSymbolStream(grid);
	check(stream.symbols.empty(), "toSymbolStream: empty grid produces an empty SymbolStream");
	Grid rebuilt = fromSymbolStream(stream, 0, 0);
	check(rebuilt.data.empty(), "fromSymbolStream: empty SymbolStream rebuilds to an empty grid");
}

void test_symbolStream_equality() {
	SymbolStream a{StreamFormat::Raw, {1, 2, 3}};
	SymbolStream b{StreamFormat::Raw, {1, 2, 3}};
	SymbolStream differentSymbols{StreamFormat::Raw, {1, 2, 4}};
	SymbolStream differentFormat{StreamFormat::RLE, {1, 2, 3}};
	check(a == b, "SymbolStream::operator==: identical format and symbols compare equal");
	check(!(a == differentSymbols), "SymbolStream::operator==: differing symbols compare unequal");
	check(!(a == differentFormat), "SymbolStream::operator==: differing format compares unequal");
}

void test_splitGrid_rebuildTiledGrid_roundTrip() {
	Grid grid = createGrid(23, 17);
	for (size_t i = 0; i < grid.data.size(); i++) {
		grid.data[i] = static_cast<int>(i % 7);
	}
	std::vector<Tile> tiles = splitGrid(grid);
	Grid rebuilt = rebuildTiledGrid(tiles, grid.rows, grid.cols);
	check(rebuilt.data == grid.data, "splitGrid/rebuildTiledGrid: round trip on non-multiple-of-tileSize grid");
}

void test_splitGrid_tileGeometry() {
	// 23x17 grid with tileSize=10 -> ceil(23/10)=3 rows of tiles, ceil(17/10)=2 cols of tiles -> 6 tiles,
	// with ragged edge tiles clipped to what's left over (3 rows, 7 cols) rather than the full 10x10.
	Grid grid = createGrid(23, 17);
	std::vector<Tile> tiles = splitGrid(grid);
	checkEqual(tiles.size(), static_cast<size_t>(6), "splitGrid: tile count for 23x17 grid with tileSize=10");

	const Tile& first = tiles.front();
	bool firstOk = first.rowStart == 0 && first.colStart == 0 && first.rows == 10 && first.cols == 10;
	check(firstOk, "splitGrid: first tile is a full 10x10 tile at the origin");

	const Tile& last = tiles.back();
	bool lastOk = last.rowStart == 20 && last.colStart == 10 && last.rows == 3 && last.cols == 7;
	check(lastOk, "splitGrid: last tile is clipped to the grid's ragged edge (3x7)");
}

void test_splitGrid_exactMultiple() {
	Grid grid = createGrid(20, 20);
	std::vector<Tile> tiles = splitGrid(grid);
	checkEqual(tiles.size(), static_cast<size_t>(4), "splitGrid: 20x20 grid with tileSize=10 produces exactly 4 tiles");
	bool allFull = true;
	for (const Tile& t : tiles) if (t.rows != 10 || t.cols != 10) allFull = false;
	check(allFull, "splitGrid: all tiles are full 10x10 when the grid divides evenly");
}

void test_updateTiles() {
	Grid grid = createGrid(10, 10);
	Tile patch;
	patch.rowStart = 2;
	patch.colStart = 3;
	patch.rows = 2;
	patch.cols = 2;
	patch.data = {9, 9, 9, 9};

	Grid updated = updateTiles(grid, {patch});

	bool patchedCorrectly =
		updated.data[2 * 10 + 3] == 9 && updated.data[2 * 10 + 4] == 9 &&
		updated.data[3 * 10 + 3] == 9 && updated.data[3 * 10 + 4] == 9;

	int nonZeroOutsidePatch = 0;
	for (int r = 0; r < updated.rows; r++) {
		for (int c = 0; c < updated.cols; c++) {
			bool inPatch = r >= patch.rowStart && r < patch.rowStart + patch.rows &&
			               c >= patch.colStart && c < patch.colStart + patch.cols;
			if (!inPatch && updated.data[r * updated.cols + c] != 0) nonZeroOutsidePatch++;
		}
	}

	check(patchedCorrectly && nonZeroOutsidePatch == 0, "updateTiles: only the patched region changes");
}

void test_updateTiles_multipleTiles() {
	Grid grid = createGrid(10, 10);

	Tile patchA;
	patchA.rowStart = 0;
	patchA.colStart = 0;
	patchA.rows = 2;
	patchA.cols = 2;
	patchA.data = {1, 1, 1, 1};

	Tile patchB;
	patchB.rowStart = 5;
	patchB.colStart = 5;
	patchB.rows = 2;
	patchB.cols = 2;
	patchB.data = {2, 2, 2, 2};

	Grid updated = updateTiles(grid, {patchA, patchB});

	bool patchAOk = updated.data[0 * 10 + 0] == 1 && updated.data[0 * 10 + 1] == 1 &&
	                updated.data[1 * 10 + 0] == 1 && updated.data[1 * 10 + 1] == 1;
	bool patchBOk = updated.data[5 * 10 + 5] == 2 && updated.data[5 * 10 + 6] == 2 &&
	                updated.data[6 * 10 + 5] == 2 && updated.data[6 * 10 + 6] == 2;

	check(patchAOk && patchBOk, "updateTiles: multiple changed tiles each apply to their own region independently");
}

void test_updateTiles_emptyChangedTiles() {
	Grid grid = createGrid(5, 5);
	Grid updated = updateTiles(grid, {});
	check(updated.data == grid.data, "updateTiles: empty changedTiles leaves grid unchanged");
}

void test_rebuildGrid() {
	std::vector<uint8_t> flat = {1, 2, 3, 4, 5, 6};
	Grid grid = rebuildGrid(flat, 2, 3);
	checkEqual(grid.rows, 2, "rebuildGrid: rows");
	checkEqual(grid.cols, 3, "rebuildGrid: cols");
	check(grid.data == flat, "rebuildGrid: data matches input flat vector");
}

void test_compressionRatio() {
	// Deliberately chosen so the true ratio is NOT a whole number: {1,1,1,2,2,3,3} RLE-encodes to
	// 3 runs from 7 elements -> 28 bytes / 24 bytes = 1.1666..., not something integer division could
	// fake. (An earlier version of this test used 100 identical values -> a single run -> a 400/8=50.0
	// ratio that divides evenly, which silently passed even when compressionRatio's implementation used
	// truncating integer division instead of floating-point division.)
	std::vector<uint8_t> uncompressed = {1, 1, 1, 2, 2, 3, 3};
	RLERuns compressed = rleEncode(uncompressed);
	size_t compressedBytes = compressed.values.size() * sizeof(uint8_t) + compressed.counts.size() * sizeof(uint16_t);
	double expected = static_cast<double>(uncompressed.size() * sizeof(Cell)) /
	                   static_cast<double>(compressedBytes);
	double actual = compressionRatio(compressedBytes, uncompressed.size() * sizeof(Cell));
	checkEqual(compressed.values.size(), static_cast<size_t>(3), "compressionRatio: setup sanity - three runs for {1,1,1,2,2,3,3}");
	checkEqual(actual, expected, "compressionRatio: matches originalSize/compressedSize for a non-integer ratio");
}

void test_compressionRatio_emptyCompressed() {
	std::vector<uint8_t> uncompressed = {1, 2, 3};
	RLERuns compressed;
	size_t compressedBytes = compressed.values.size() * sizeof(uint8_t) + compressed.counts.size() * sizeof(uint16_t);
	checkEqual(compressionRatio(compressedBytes, uncompressed.size() * sizeof(Cell)), 0.0,
	           "compressionRatio: empty compressed input returns 0.0");
}

int main() {
	test_createGrid();
	test_rleEncodeDecode_roundTrip();
	test_rleEncode_singleRun();
	test_rleEncodeDecode_empty();
	test_rleDecode_mismatchedLengthsRejected();
	test_bitWriter_bitReader_roundTrip();
	test_bitWriter_bitReader_nonByteAlignedTrailingBits();
	test_bitReader_truncatedStreamRejected();
	test_varint_roundTrip_variousValues();
	test_varint_decode_truncatedStreamRejected();
	test_varint_decode_excessContinuationBytesRejected();
	test_fixedWidthCoder_roundTrip();
	test_fixedWidthCoder_rejectsOutOfRangeValue();
	test_fixedWidthCoder_decode_truncatedRejected();
	test_varintCoder_roundTrip();
	test_varintCoder_decode_overflowRejected();
	test_countsCoder_dispatchesVarint();
	test_countsCoder_dispatchesRice();
	test_rice_encode_rejectsShiftWidthOverflow();
	test_rice_decode_rejectsShiftWidthOverflow();
	test_rlecodec_encode_rejectsRiceShiftWidthOverflow();
	test_splitcodec_encode_rejectsRiceShiftWidthOverflow();
	test_rlecodec_decode_rejectsAdversarialRiceParam();
	test_splitcodec_decode_rejectsAdversarialRiceParam();
	test_countsCoder_chooseBest_picksSmallerAndRoundTrips();
	test_rleCodec_autoSelect_embedsWinningCoderType();
	test_rleCodec_decode_rejectsUnrecognizedCoderType();
	test_rleCodec_roundTrip();
	test_rleCodec_rejectsMismatchedRuns();
	test_rleCodec_rejectsValueOutOfBitWidth();
	test_rleCodec_decode_truncatedRejected();
	test_fullPipeline_roundTrip_emptyGrid();
	test_fullPipeline_roundTrip_singleRun();
	test_fullPipeline_roundTrip_worstCaseCheckerboard();
	test_packetizer_fragment_smallData();
	test_packetizer_fragment_emptyData();
	test_packetizer_fragment_multipleFragments_reassembleByPosition();
	test_packetizer_serialize_deserialize_roundTrip();
	test_packetizer_deserialize_rejectsTruncated();
	test_packetizer_deserialize_rejectsCorruptedCrc();
	test_packetizer_deserialize_rejectsPayloadLengthMismatch();
	test_loopbackTransport_receiveEmptyReturnsFalse();
	test_loopbackTransport_sendReceive_fifoOrder();
	test_loopbackTransport_carriesPacketizerBytes();
	test_reassembler_singleFragment_completeImmediately();
	test_reassembler_multipleFragments_outOfOrder_reassembleCorrectly();
	test_reassembler_incomplete_returnsFalse();
	test_reassembler_duplicateFragment_idempotent();
	test_reassembler_inconsistentTotalFragments_rejected();
	test_reassembler_fragmentIndexOutOfRange_rejected();
	test_reassembler_missingFragments_reportsCorrectIndices();
	test_reassembler_independentStreamsByKey();
	test_splitCodec_roundTrip();
	test_splitCodec_explicitChoice_forcesRequestedCoder();
	test_splitCodec_rejectsMismatchedRuns();
	test_splitCodec_decode_rejectsDisagreeingStreams();
	test_lossyTransport_zeroDropProbability_neverDrops();
	test_lossyTransport_oneDropProbability_alwaysDrops();
	test_lossyTransport_partialLoss_conservesCount();
	test_arq_drainInto_reliableTransport_deliversEverything();
	test_arq_sendWithRetry_recoversFromPartialLoss();
	test_arq_sendWithRetry_givesUpAfterMaxRetries();
	test_toSymbolStream_fromSymbolStream_roundTrip();
	test_toSymbolStream_tagsRaw();
	test_toSymbolStream_fromSymbolStream_emptyGrid();
	test_symbolStream_equality();
	test_splitGrid_rebuildTiledGrid_roundTrip();
	test_splitGrid_tileGeometry();
	test_splitGrid_exactMultiple();
	test_updateTiles();
	test_updateTiles_multipleTiles();
	test_updateTiles_emptyChangedTiles();
	test_rebuildGrid();
	test_compressionRatio();
	test_compressionRatio_emptyCompressed();

	if (failures == 0) {
		std::cout << "All tests passed.\n";
	} else {
		std::cout << failures << " test(s) failed.\n";
	}
	return failures == 0 ? 0 : 1;
}
