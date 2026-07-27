#include "rle.h"
#include "grid.h"
#include "metrics.h"

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
	SymbolStream data = {1, 1, 1, 0, 0, 2, 2, 2, 2};
	std::vector<RLEbits> encoded = rleEncode(data.symbol);
	std::vector<uint8_t> decoded = rleDecode(encoded);
	checkEqual(encoded.size(), static_cast<size_t>(3), "rleEncode: run count for {1,1,1,0,0,2,2,2,2}");
	check(decoded == data, "rleEncode/rleDecode: round trip preserves data");
}

void test_rleEncode_singleRun() {
	std::vector<uint8_t> data = {5, 5, 5, 5};
	std::vector<RLEbits> encoded = rleEncode(data);
	bool ok = encoded.size() == 1 && encoded[0].value == 5 && encoded[0].count == 4;
	check(ok, "rleEncode: single run collapses to one RLEbits{value=5,count=4}");
}

void test_rleEncodeDecode_empty() {
	std::vector<uint8_t> empty;
	std::vector<RLEbits> encoded = rleEncode(empty);
	std::vector<uint8_t> decoded = rleDecode(encoded);
	check(encoded.empty(), "rleEncode: empty input produces empty output");
	check(decoded.empty(), "rleDecode: empty input produces empty output");
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
	std::vector<RLEbits> compressed = rleEncode(uncompressed);
	double expected = static_cast<double>(uncompressed.size() * sizeof(Cell)) /
	                   static_cast<double>(compressed.size() * sizeof(RLEbits));
	double actual = compressionRatio(compressed.size() * sizeof(RLEbits), uncompressed.size() * sizeof(Cell));
	checkEqual(compressed.size(), static_cast<size_t>(3), "compressionRatio: setup sanity - three runs for {1,1,1,2,2,3,3}");
	checkEqual(actual, expected, "compressionRatio: matches originalSize/compressedSize for a non-integer ratio");
}

void test_compressionRatio_emptyCompressed() {
	std::vector<uint8_t> uncompressed = {1, 2, 3};
	std::vector<RLEbits> compressed;
	checkEqual(compressionRatio(compressed.size() * sizeof(RLEbits), uncompressed.size() * sizeof(Cell)), 0.0,
	           "compressionRatio: empty compressed input returns 0.0");
}

int main() {
	test_createGrid();
	test_rleEncodeDecode_roundTrip();
	test_rleEncode_singleRun();
	test_rleEncodeDecode_empty();
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
