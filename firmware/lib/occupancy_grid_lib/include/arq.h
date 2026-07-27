#ifndef ARQ_H
#define ARQ_H

#include <cstdint>
#include <vector>
#include "transport.h"
#include "packet_reassembler.h"

namespace compressor{
	// Batched per-message ARQ: send every fragment once, then retry only
	// the fragments the reassembler reports missing, up to a retry limit,
	// rather than resending the whole message on any loss. Balances
	// fault-containment precision against retransmission cost.
	namespace arq{
		struct Result{
			bool success = false;
			int roundsUsed = 0;
			size_t packetsSentTotal = 0;
		};

		// Drains whatever's currently sitting in the transport into the
		// reassembler. Malformed/corrupted packets (failed CRC, etc.) are
		// silently dropped -- their fragmentIndex will simply show up in
		// missingFragments() on the next round, same as if it were lost
		// in transit.
		void drainInto(ITransport& transport, PacketReassembler& reassembler);

		// Fragments `data`, sends every fragment once, drains the
		// transport, then repeats "resend exactly what's still missing" up
		// to maxRetries additional rounds. Returns once the stream is
		// complete or retries are exhausted -- success reflects which.
		Result sendWithRetry(ITransport& transport, PacketReassembler& reassembler,
		                     uint16_t messageId, uint8_t streamId,
		                     const std::vector<uint8_t>& data, int maxRetries);
	}
}
#endif
