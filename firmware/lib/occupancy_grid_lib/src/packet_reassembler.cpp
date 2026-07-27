#include "packet_reassembler.h"

namespace compressor{
	uint32_t PacketReassembler::makeKey(uint16_t messageId, uint8_t streamId){
		return (static_cast<uint32_t>(messageId) << 8) | static_cast<uint32_t>(streamId);
	}

	bool PacketReassembler::receive(const packetizer::Packet& packet){
		uint32_t key = makeKey(packet.header.messageId, packet.header.streamId);
		StreamState& state = streams_[key];

		if(state.totalFragmentsKnown && state.totalFragments != packet.header.totalFragments){
			return false; // disagrees with what earlier fragments of this same message/stream reported
		}
		if(packet.header.fragmentIndex >= packet.header.totalFragments){
			return false; // malformed: index out of the range its own header declares
		}

		state.totalFragments = packet.header.totalFragments;
		state.totalFragmentsKnown = true;
		state.fragments[packet.header.fragmentIndex] = packet.payload; // duplicates overwrite idempotently
		return true;
	}

	bool PacketReassembler::isComplete(uint16_t messageId, uint8_t streamId) const{
		auto it = streams_.find(makeKey(messageId, streamId));
		if(it == streams_.end()) return false;
		return it->second.totalFragmentsKnown && it->second.fragments.size() == it->second.totalFragments;
	}

	bool PacketReassembler::tryGetCompleteStream(uint16_t messageId, uint8_t streamId, std::vector<uint8_t>& out) const{
		if(!isComplete(messageId, streamId)) return false;

		const StreamState& state = streams_.at(makeKey(messageId, streamId));
		std::vector<uint8_t> result;
		for(uint16_t i = 0; i < state.totalFragments; i++){
			const std::vector<uint8_t>& payload = state.fragments.at(i);
			result.insert(result.end(), payload.begin(), payload.end());
		}
		out = std::move(result);
		return true;
	}

	std::vector<uint16_t> PacketReassembler::missingFragments(uint16_t messageId, uint8_t streamId) const{
		std::vector<uint16_t> missing;
		auto it = streams_.find(makeKey(messageId, streamId));
		if(it == streams_.end() || !it->second.totalFragmentsKnown) return missing;

		const StreamState& state = it->second;
		for(uint16_t i = 0; i < state.totalFragments; i++){
			if(state.fragments.find(i) == state.fragments.end()) missing.push_back(i);
		}
		return missing;
	}
}
