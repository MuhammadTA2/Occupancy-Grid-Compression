#include "loopback_transport.h"

namespace compressor{
	bool LoopbackTransport::send(const std::vector<uint8_t>& data){
		queue_.push_back(data);
		return true;
	}

	bool LoopbackTransport::receive(std::vector<uint8_t>& out){
		if(queue_.empty()) return false;
		out = std::move(queue_.front());
		queue_.pop_front();
		return true;
	}

	size_t LoopbackTransport::pending() const{
		return queue_.size();
	}
}
