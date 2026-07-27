#include "arq.h"
#include "packetizer.h"

namespace compressor{
	namespace arq{
		void drainInto(ITransport& transport, PacketReassembler& reassembler){
			std::vector<uint8_t> raw;
			while(transport.receive(raw)){
				packetizer::Packet p;
				if(packetizer::deserialize(raw, p)){
					reassembler.receive(p);
				}
				// a packet that fails to deserialize (corrupted, truncated)
				// is dropped here -- its fragmentIndex reappears in
				// missingFragments() next round, same as a packet lost
				// outright by the transport.
			}
		}

		Result sendWithRetry(ITransport& transport, PacketReassembler& reassembler,
		                     uint16_t messageId, uint8_t streamId,
		                     const std::vector<uint8_t>& data, int maxRetries){
			Result result;
			std::vector<packetizer::Packet> allPackets = packetizer::fragment(messageId, streamId, data);

			for(const packetizer::Packet& p : allPackets){
				transport.send(packetizer::serialize(p));
				result.packetsSentTotal++;
			}
			drainInto(transport, reassembler);
			result.roundsUsed = 1;

			while(!reassembler.isComplete(messageId, streamId) && result.roundsUsed <= maxRetries){
				std::vector<uint16_t> missing = reassembler.missingFragments(messageId, streamId);
				if(missing.empty()){
					// The reassembler doesn't know about this stream at all
					// yet -- every fragment sent so far was lost before
					// arriving, so it never learned totalFragments in the
					// first place. missingFragments() can't report anything
					// in that case, so fall back to our own authoritative
					// fragment list and resend everything, rather than
					// mistaking "reassembler knows of nothing missing" for
					// "nothing left to do."
					missing.reserve(allPackets.size());
					for(size_t i = 0; i < allPackets.size(); i++) missing.push_back(static_cast<uint16_t>(i));
				}

				for(uint16_t idx : missing){
					transport.send(packetizer::serialize(allPackets[idx]));
					result.packetsSentTotal++;
				}
				drainInto(transport, reassembler);
				result.roundsUsed++;
			}

			result.success = reassembler.isComplete(messageId, streamId);
			return result;
		}
	}
}
