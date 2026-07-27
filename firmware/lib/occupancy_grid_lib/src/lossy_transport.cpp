#include "lossy_transport.h"

namespace compressor{
	LossyTransport::LossyTransport(double dropProbability, unsigned seed)
		: dropProbability_(dropProbability), rng_(seed), dist_(0.0, 1.0) {}

	bool LossyTransport::send(const std::vector<uint8_t>& data){
		if(dist_(rng_) < dropProbability_){
			droppedCount_++;
			return true; // the sender has no synchronous way to know this was lost
		}
		queue_.push_back(data);
		return true;
	}

	bool LossyTransport::receive(std::vector<uint8_t>& out){
		if(queue_.empty()) return false;
		out = std::move(queue_.front());
		queue_.pop_front();
		return true;
	}

	size_t LossyTransport::pending() const{
		return queue_.size();
	}

	size_t LossyTransport::droppedCount() const{
		return droppedCount_;
	}
}
