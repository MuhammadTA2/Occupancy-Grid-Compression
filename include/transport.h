#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <cstdint>
#include <vector>

namespace compressor{
	// Transport-agnostic byte-vector-in/byte-vector-out interface. This is
	// one of the few places a real virtual interface fits the project: the
	// compression layer uses tag-dispatch (enums + switch) specifically to
	// avoid vtables on the ESP32 target, because that dispatch happens per
	// symbol; transport swapping happens per packet, so the vtable overhead
	// here is negligible, and swapping LoopbackTransport for a real
	// LoRaTransport later (Milestone 12) is exactly the seam a virtual
	// interface is good for.
	class ITransport{
	public:
		virtual ~ITransport() = default;

		// Sends one already-serialized packet's worth of bytes (e.g. the
		// output of packetizer::serialize). Returns false if the transport
		// can't accept it right now.
		virtual bool send(const std::vector<uint8_t>& data) = 0;

		// Attempts to receive one packet's worth of bytes. Returns false if
		// nothing is available right now -- not an error, just "nothing yet".
		virtual bool receive(std::vector<uint8_t>& out) = 0;
	};
}
#endif
