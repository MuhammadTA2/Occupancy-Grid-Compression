#include "rle.h"
#include "grid.h"
#include "metrics.h"
#include "rle_codec.h"
#include "split_codec.h"
#include "packetizer.h"
#include "loopback_transport.h"
#include "packet_reassembler.h"
#include "lossy_transport.h"
#include "arq.h"

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

	RLERuns encoded= rleEncode(grid_tx.data);
	std::vector<uint8_t> decoded= rleDecode(encoded);
	Grid grid_rx= rebuildGrid(decoded,grid_tx.rows,grid_tx.cols);
			

	


	
	
	size_t structuralBytes = encoded.values.size()*sizeof(uint8_t) + encoded.counts.size()*sizeof(uint16_t);
	double cRatio=compressionRatio(structuralBytes,grid_tx.data.size()*sizeof(Cell));

 	std::cout << "Original cells: " << grid_tx.data.size() << "\n";
    	std::cout << "Encoded RLE runs: " << encoded.values.size() << "\n";
    	std::cout << "Decoded cells: " << decoded.size() << "\n";
    	std::cout << "Rebuilt grid matches original: " << (grid_tx.data==grid_rx.data ? "YES" : "NO") << "\n";
    	std::cout << "Compression ratio: " << cRatio<< "\n";

    // Tile test
    	std::vector<Tile> tiles = splitGrid(grid_tx);
    	Grid tile_rebuilt = rebuildTiledGrid(tiles, grid_tx.rows, grid_tx.cols);

    	std::cout << "Tile count: " << tiles.size() << "\n";
    	std::cout << "Tile rebuild matches original: " << ((tile_rebuilt.data == grid_tx.data) ? "YES" : "NO") << "\n";

	// SymbolStream conversion test
	SymbolStream stream = toSymbolStream(grid_tx);
	Grid grid_from_stream = fromSymbolStream(stream, grid_tx.rows, grid_tx.cols);

	std::cout << "SymbolStream round trip matches original: " << ((grid_from_stream.data == grid_tx.data) ? "YES" : "NO") << "\n";

	// Milestone 4/5 pipeline: SymbolStream -> RLE -> FixedWidthCoder (values)
	// + auto-selected counts coder (Varint or Rice, whichever is smaller for
	// this payload) -> bytes -> back to Grid.
	const uint8_t valueBitWidth = 2; // occupancy values are 0/1/2, 11 reserved
	std::vector<uint8_t> pipelineBlob = rlecodec::encode(encoded, valueBitWidth);
	CoderType chosenCountsCoder = pipelineBlob.empty() ? CoderType::Unknown : static_cast<CoderType>(pipelineBlob[0]);
	const char* chosenCountsCoderName =
		chosenCountsCoder == CoderType::Varint ? "Varint" :
		chosenCountsCoder == CoderType::Rice ? "Rice" : "Unknown";

	RLERuns pipelineDecodedRuns;
	bool pipelineDecodeOk = rlecodec::decode(pipelineBlob, valueBitWidth, pipelineDecodedRuns);
	std::vector<uint8_t> pipelineDecodedSymbols = rleDecode(pipelineDecodedRuns);
	Grid grid_pipeline = fromSymbolStream(SymbolStream{StreamFormat::Raw, pipelineDecodedSymbols}, grid_tx.rows, grid_tx.cols);

	double pipelineRatio = compressionRatio(pipelineBlob.size(), grid_tx.data.size() * sizeof(Cell));

	std::cout << "\n--- Milestone 4/5 pipeline (FixedWidthCoder + auto-selected counts coder) ---\n";
	std::cout << "Counts coder chosen for this payload: " << chosenCountsCoderName << "\n";
	std::cout << "Encoded blob size (bytes): " << pipelineBlob.size() << "\n";
	std::cout << "Pipeline decode succeeded: " << (pipelineDecodeOk ? "YES" : "NO") << "\n";
	std::cout << "Pipeline round trip matches original: " << ((pipelineDecodeOk && grid_pipeline.data == grid_tx.data) ? "YES" : "NO") << "\n";
	std::cout << "Pipeline compression ratio (vs raw cells): " << pipelineRatio << "\n";
	std::cout << "Structural (values+counts) compression ratio was: " << cRatio << "\n";

	// Milestone 10: full loopback pipeline. Grid -> SymbolStream -> RLE ->
	// splitcodec (two independent streams) -> Packetizer (fragmented
	// separately per stream, for fault containment) -> LoopbackTransport ->
	// PacketReassembler -> splitcodec::decode -> RLE rebuild -> SymbolStream
	// -> Grid. This is the first true end-to-end demonstration of the whole
	// documented pipeline, not just its pieces tested in isolation.
	std::cout << "\n--- Milestone 10: full loopback pipeline (Packetizer + LoopbackTransport + PacketReassembler) ---\n";
	{
		const uint16_t messageId = 1;
		const uint8_t VALUES_STREAM_ID = 0;
		const uint8_t COUNTS_STREAM_ID = 1;

		// Sender side.
		splitcodec::EncodedStreams txStreams = splitcodec::encode(encoded, valueBitWidth);
		std::vector<packetizer::Packet> valuesPackets = packetizer::fragment(messageId, VALUES_STREAM_ID, txStreams.valuesBytes);
		std::vector<packetizer::Packet> countsPackets = packetizer::fragment(messageId, COUNTS_STREAM_ID, txStreams.countsBytes);

		LoopbackTransport transport;
		for(const auto& p : valuesPackets) transport.send(packetizer::serialize(p));
		for(const auto& p : countsPackets) transport.send(packetizer::serialize(p));

		size_t packetsSent = valuesPackets.size() + countsPackets.size();

		// Receiver side.
		PacketReassembler reassembler;
		size_t packetsReceived = 0;
		size_t packetsRejected = 0;
		std::vector<uint8_t> raw;
		while(transport.receive(raw)){
			packetizer::Packet decodedPacket;
			if(packetizer::deserialize(raw, decodedPacket) && reassembler.receive(decodedPacket)){
				packetsReceived++;
			} else {
				packetsRejected++;
			}
		}

		bool valuesComplete = reassembler.isComplete(messageId, VALUES_STREAM_ID);
		bool countsComplete = reassembler.isComplete(messageId, COUNTS_STREAM_ID);

		std::vector<uint8_t> rxValuesBytes, rxCountsBytes;
		bool gotValues = reassembler.tryGetCompleteStream(messageId, VALUES_STREAM_ID, rxValuesBytes);
		bool gotCounts = reassembler.tryGetCompleteStream(messageId, COUNTS_STREAM_ID, rxCountsBytes);

		// Decode-and-validate.
		RLERuns rxRuns;
		bool decodeOk = gotValues && gotCounts && splitcodec::decode(rxValuesBytes, rxCountsBytes, valueBitWidth, rxRuns);
		std::vector<uint8_t> rxSymbols = rleDecode(rxRuns);
		Grid grid_loopback = fromSymbolStream(SymbolStream{StreamFormat::Raw, rxSymbols}, grid_tx.rows, grid_tx.cols);

		std::cout << "Packets sent (values + counts): " << valuesPackets.size() << " + " << countsPackets.size()
		           << " = " << packetsSent << "\n";
		std::cout << "Packets received/rejected by the reassembler: " << packetsReceived << " / " << packetsRejected << "\n";
		std::cout << "Both streams complete after reassembly: " << ((valuesComplete && countsComplete) ? "YES" : "NO") << "\n";
		std::cout << "Decode-and-validate succeeded: " << (decodeOk ? "YES" : "NO") << "\n";
		std::cout << "Full loopback pipeline round trip matches original: "
		           << ((decodeOk && grid_loopback.data == grid_tx.data) ? "YES" : "NO") << "\n";
	}

	// Milestone 11: same pipeline, but over a deliberately lossy transport
	// (30% drop rate), recovered via batched per-message ARQ instead of a
	// reliable LoopbackTransport.
	std::cout << "\n--- Milestone 11: ARQ over a lossy transport (30% drop rate) ---\n";
	{
		const uint16_t messageId = 2;
		const uint8_t VALUES_STREAM_ID = 0;
		const uint8_t COUNTS_STREAM_ID = 1;
		const int maxRetries = 10;

		splitcodec::EncodedStreams txStreams = splitcodec::encode(encoded, valueBitWidth);

		LossyTransport transport(0.3);
		PacketReassembler reassembler;

		arq::Result valuesResult = arq::sendWithRetry(transport, reassembler, messageId, VALUES_STREAM_ID, txStreams.valuesBytes, maxRetries);
		arq::Result countsResult = arq::sendWithRetry(transport, reassembler, messageId, COUNTS_STREAM_ID, txStreams.countsBytes, maxRetries);

		std::vector<uint8_t> rxValuesBytes, rxCountsBytes;
		bool gotValues = reassembler.tryGetCompleteStream(messageId, VALUES_STREAM_ID, rxValuesBytes);
		bool gotCounts = reassembler.tryGetCompleteStream(messageId, COUNTS_STREAM_ID, rxCountsBytes);

		RLERuns rxRuns;
		bool decodeOk = gotValues && gotCounts && splitcodec::decode(rxValuesBytes, rxCountsBytes, valueBitWidth, rxRuns);
		std::vector<uint8_t> rxSymbols = rleDecode(rxRuns);
		Grid grid_arq = fromSymbolStream(SymbolStream{StreamFormat::Raw, rxSymbols}, grid_tx.rows, grid_tx.cols);

		std::cout << "Values stream: " << valuesResult.roundsUsed << " round(s), "
		           << valuesResult.packetsSentTotal << " packet sends, success=" << (valuesResult.success ? "YES" : "NO") << "\n";
		std::cout << "Counts stream: " << countsResult.roundsUsed << " round(s), "
		           << countsResult.packetsSentTotal << " packet sends, success=" << (countsResult.success ? "YES" : "NO") << "\n";
		std::cout << "Packets actually dropped by the transport: " << transport.droppedCount() << "\n";
		std::cout << "Decode-and-validate succeeded: " << (decodeOk ? "YES" : "NO") << "\n";
		std::cout << "ARQ pipeline round trip matches original: "
		           << ((decodeOk && grid_arq.data == grid_tx.data) ? "YES" : "NO") << "\n";
	}

	return 0;

}
