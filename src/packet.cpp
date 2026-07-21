// PROTOTYPE -- see include/packet.h and Discussion.md (07/20/2026 entry).
// Not wired into CMakeLists.txt; expected to be replaced, not extended,
// when Milestone 6 builds Packet against real IEntropyCoder bytes.
#include "packet.h"
#include <algorithm>

namespace compressor{

	std::vector<uint8_t> serializeRLE(const std::vector<RLEbits>& runs){
		std::vector<uint8_t> bytes;
		bytes.reserve(runs.size()*kRLERecordBytes);
		for(const auto& run : runs){
			bytes.push_back(run.value);
			bytes.push_back(static_cast<uint8_t>(run.count & 0xFF));         // low byte
			bytes.push_back(static_cast<uint8_t>((run.count >> 8) & 0xFF));  // high byte
		}
		return bytes;
	}

	std::vector<RLEbits> deserializeRLE(const std::vector<uint8_t>& bytes){
		std::vector<RLEbits> runs;
		if(bytes.size() % kRLERecordBytes != 0) return runs; // truncated/corrupt input
		runs.reserve(bytes.size()/kRLERecordBytes);
		for(size_t i=0; i<bytes.size(); i+=kRLERecordBytes){
			RLEbits run;
			run.value = bytes[i];
			run.count = static_cast<uint16_t>(bytes[i+1] | (bytes[i+2] << 8));
			runs.push_back(run);
		}
		return runs;
	}

	uint8_t xorChecksum(const std::vector<uint8_t>& data){
		uint8_t sum = 0;
		for(uint8_t b : data) sum ^= b;
		return sum;
	}

	std::vector<Packet> packetizeTile(uint8_t tileId, const std::vector<uint8_t>& serializedRLE, uint8_t maxPayload){
		std::vector<Packet> packets;
		if(serializedRLE.empty() || maxPayload==0) return packets;

		size_t numPackets = (serializedRLE.size() + maxPayload - 1) / maxPayload;

		for(size_t seq=0; seq<numPackets; seq++){
			size_t start = seq*maxPayload;
			size_t len = std::min(static_cast<size_t>(maxPayload), serializedRLE.size()-start);

			Packet pkt;
			pkt.payload.assign(serializedRLE.begin()+start, serializedRLE.begin()+start+len);
			pkt.header.tileId = tileId;
			pkt.header.seq = static_cast<uint8_t>(seq);
			pkt.header.totalSeqs = static_cast<uint8_t>(numPackets);
			pkt.header.payloadLen = static_cast<uint16_t>(len);
			pkt.header.checksum = xorChecksum(pkt.payload);

			packets.push_back(std::move(pkt));
		}
		return packets;
	}

	std::vector<uint8_t> reassembleTile(const std::vector<Packet>& packets){
		std::vector<uint8_t> out;
		for(const auto& pkt : packets){
			out.insert(out.end(), pkt.payload.begin(), pkt.payload.end());
		}
		return out;
	}

}
