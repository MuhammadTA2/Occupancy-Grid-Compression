#ifndef LOSSY_TRANSPORT_H
#define LOSSY_TRANSPORT_H

#include "transport.h"
#include <deque>
#include <random>

namespace compressor{
	// Test double that deliberately drops packets at a configurable rate,
	// standing in for a noisy LoRa/UART link. send() always returns true --
	// a real sender has no synchronous way to know a transmission was lost,
	// so this doesn't either; whether something got through is only visible
	// later, from the receiver's missing-fragment report. Reordering and
	// duplication aren't separately modeled here: PacketReassembler already
	// tolerates both (Milestone 9's tests cover that directly), so the one
	// behavior worth a dedicated test double is loss, which is what ARQ
	// actually exists to recover from.
	class LossyTransport : public ITransport{
	public:
		// dropProbability in [0, 1]. seed fixed by default so tests are
		// reproducible; pass a different seed to get different loss patterns.
		explicit LossyTransport(double dropProbability, unsigned seed = 12345);

		bool send(const std::vector<uint8_t>& data) override;
		bool receive(std::vector<uint8_t>& out) override;

		size_t pending() const;
		size_t droppedCount() const;

	private:
		double dropProbability_;
		std::mt19937 rng_;
		std::uniform_real_distribution<double> dist_;
		std::deque<std::vector<uint8_t>> queue_;
		size_t droppedCount_ = 0;
	};
}
#endif
