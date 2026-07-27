#include "packetizer.h"
#include <algorithm>

namespace compressor{
	namespace packetizer{
		uint16_t crc16(const std::vector<uint8_t>& data){
			uint16_t crc = 0xFFFF;
			for(uint8_t byte : data){
				crc ^= static_cast<uint16_t>(byte) << 8;
				for(int i = 0; i < 8; i++){
					if(crc & 0x8000u) crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
					else crc = static_cast<uint16_t>(crc << 1);
				}
			}
			return crc;
		}

		std::vector<Packet> fragment(uint16_t messageId, uint8_t streamId, const std::vector<uint8_t>& data){
			std::vector<Packet> packets;

			size_t totalFragments = data.empty() ? 1 : (data.size() + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

			for(size_t i = 0; i < totalFragments; i++){
				size_t start = i * MAX_PAYLOAD_SIZE;
				size_t len = data.empty() ? 0 : std::min(MAX_PAYLOAD_SIZE, data.size() - start);

				Packet p;
				p.header.version = 1;
				p.header.messageId = messageId;
				p.header.streamId = streamId;
				p.header.fragmentIndex = static_cast<uint16_t>(i);
				p.header.totalFragments = static_cast<uint16_t>(totalFragments);
				p.header.payloadLength = static_cast<uint8_t>(len);
				p.payload.assign(data.begin() + static_cast<long>(start), data.begin() + static_cast<long>(start + len));
				packets.push_back(std::move(p));
			}

			return packets;
		}

		std::vector<uint8_t> serialize(const Packet& packet){
			std::vector<uint8_t> out;
			out.reserve(HEADER_SIZE + packet.payload.size() + CRC_SIZE);

			out.push_back(packet.header.version);
			out.push_back(static_cast<uint8_t>(packet.header.messageId & 0xFFu));
			out.push_back(static_cast<uint8_t>((packet.header.messageId >> 8) & 0xFFu));
			out.push_back(packet.header.streamId);
			out.push_back(static_cast<uint8_t>(packet.header.fragmentIndex & 0xFFu));
			out.push_back(static_cast<uint8_t>((packet.header.fragmentIndex >> 8) & 0xFFu));
			out.push_back(static_cast<uint8_t>(packet.header.totalFragments & 0xFFu));
			out.push_back(static_cast<uint8_t>((packet.header.totalFragments >> 8) & 0xFFu));
			out.push_back(packet.header.payloadLength);
			out.insert(out.end(), packet.payload.begin(), packet.payload.end());

			uint16_t crc = crc16(out); // covers header+payload
			out.push_back(static_cast<uint8_t>(crc & 0xFFu));
			out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFu));
			return out;
		}

		bool deserialize(const std::vector<uint8_t>& data, Packet& out){
			if(data.size() < HEADER_SIZE + CRC_SIZE) return false; // too short even for an empty payload

			size_t crcOffset = data.size() - CRC_SIZE;
			uint16_t receivedCrc = static_cast<uint16_t>(data[crcOffset]) |
			                        static_cast<uint16_t>(static_cast<uint16_t>(data[crcOffset + 1]) << 8);

			std::vector<uint8_t> headerAndPayload(data.begin(), data.begin() + static_cast<long>(crcOffset));
			if(crc16(headerAndPayload) != receivedCrc) return false; // corrupted

			Packet p;
			p.header.version = data[0];
			p.header.messageId = static_cast<uint16_t>(data[1]) | static_cast<uint16_t>(static_cast<uint16_t>(data[2]) << 8);
			p.header.streamId = data[3];
			p.header.fragmentIndex = static_cast<uint16_t>(data[4]) | static_cast<uint16_t>(static_cast<uint16_t>(data[5]) << 8);
			p.header.totalFragments = static_cast<uint16_t>(data[6]) | static_cast<uint16_t>(static_cast<uint16_t>(data[7]) << 8);
			p.header.payloadLength = data[8];

			size_t expectedTotalLen = HEADER_SIZE + p.header.payloadLength + CRC_SIZE;
			if(data.size() != expectedTotalLen) return false; // payloadLength inconsistent with actual data length

			p.payload.assign(data.begin() + static_cast<long>(HEADER_SIZE),
			                  data.begin() + static_cast<long>(HEADER_SIZE + p.header.payloadLength));
			out = std::move(p);
			return true;
		}
	}
}
