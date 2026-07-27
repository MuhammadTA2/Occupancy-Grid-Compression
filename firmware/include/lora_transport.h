#ifndef LORA_TRANSPORT_H
#define LORA_TRANSPORT_H

#include "transport.h"

namespace compressor{
	// ITransport backed by the sandeepmistry/LoRa Arduino library
	// (SX127x-family radios, e.g. RFM95). Each send()/receive() call moves
	// exactly one packetizer::Packet's serialized bytes as one LoRa radio
	// packet -- packetizer::MAX_PACKET_SIZE (255) was already chosen with
	// the SX127x's ~256-byte FIFO in mind (see WireFormat.md section 9).
	//
	// Call LoRa.setPins()/LoRa.begin() yourself in setup() before using this
	// class -- it only wraps send/receive, not radio initialization, so a
	// failed radio init is visible at the call site (check LoRa.begin()'s
	// return value), not silently swallowed here.
	class LoRaTransport : public ITransport{
	public:
		bool send(const std::vector<uint8_t>& data) override;
		bool receive(std::vector<uint8_t>& out) override;
	};
}
#endif
