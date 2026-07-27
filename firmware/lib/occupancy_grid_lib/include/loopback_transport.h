#ifndef LOOPBACK_TRANSPORT_H
#define LOOPBACK_TRANSPORT_H

#include "transport.h"
#include <deque>
#include <cstddef>

namespace compressor{
	// In-memory FIFO queue transport for testing: send() enqueues,
	// receive() dequeues in the same order, no loss/duplication/reordering.
	// See the lossy transport test double (Milestone 11) for a transport
	// that deliberately misbehaves.
	class LoopbackTransport : public ITransport{
	public:
		bool send(const std::vector<uint8_t>& data) override;
		bool receive(std::vector<uint8_t>& out) override;

		size_t pending() const;

	private:
		std::deque<std::vector<uint8_t>> queue_;
	};
}
#endif
