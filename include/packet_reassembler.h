#ifndef PACKET_REASSEMBLER_H
#define PACKET_REASSEMBLER_H

#include <cstdint>
#include <map>
#include <vector>
#include "packetizer.h"

namespace compressor{
	// Reassembles fragmented byte streams keyed by (messageId, streamId).
	// Byte-exact by construction: fragments are concatenated strictly by
	// fragmentIndex order, never by arrival order, so a multi-byte field
	// that happened to cross a fragment boundary still reconstructs
	// correctly regardless of network reordering.
	class PacketReassembler{
	public:
		// Feeds one already-CRC-verified packet in. Returns false if it's
		// inconsistent with fragments already seen for the same key (a
		// disagreeing totalFragments, or a fragmentIndex >= totalFragments)
		// -- a genuine duplicate (same index, arrives twice) is accepted
		// idempotently, not an error.
		bool receive(const packetizer::Packet& packet);

		// True once every fragment for (messageId, streamId) has arrived.
		bool isComplete(uint16_t messageId, uint8_t streamId) const;

		// Concatenates fragment payloads in fragmentIndex order into out.
		// Returns false (leaving out unspecified) if the stream isn't
		// complete yet.
		bool tryGetCompleteStream(uint16_t messageId, uint8_t streamId, std::vector<uint8_t>& out) const;

		// Fragment indices not yet received for this key, in ascending
		// order -- the input to Milestone 11's per-message ARQ. Empty if the
		// key is unknown or already complete.
		std::vector<uint16_t> missingFragments(uint16_t messageId, uint8_t streamId) const;

	private:
		struct StreamState{
			uint16_t totalFragments = 0;
			bool totalFragmentsKnown = false;
			std::map<uint16_t, std::vector<uint8_t>> fragments; // fragmentIndex -> payload
		};

		static uint32_t makeKey(uint16_t messageId, uint8_t streamId);

		std::map<uint32_t, StreamState> streams_;
	};
}
#endif
