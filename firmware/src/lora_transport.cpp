#include "lora_transport.h"
#include <LoRa.h>

namespace compressor{
	bool LoRaTransport::send(const std::vector<uint8_t>& data){
		if(data.size() > 255) return false; // won't fit in one LoRa radio packet

		LoRa.beginPacket();
		LoRa.write(data.data(), data.size());
		LoRa.endPacket();
		return true;
	}

	bool LoRaTransport::receive(std::vector<uint8_t>& out){
		int packetSize = LoRa.parsePacket();
		if(packetSize <= 0) return false; // nothing received right now

		std::vector<uint8_t> received;
		received.reserve(static_cast<size_t>(packetSize));
		while(LoRa.available()){
			received.push_back(static_cast<uint8_t>(LoRa.read()));
		}

		out = std::move(received);
		return true;
	}
}
