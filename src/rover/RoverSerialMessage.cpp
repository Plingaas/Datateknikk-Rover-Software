//
// Created by peter on 29.10.24.
//
#include "RoverSerialMessage.hpp"

namespace Rover {
    uint8_t SerialMessage::sequence = 0;

    uint8_t SerialMessage::getSequenceByte() {
        sequence == 255 ? sequence = 0 : sequence++;
        return sequence;
    }

    uint8_t SerialMessage::checksum(const std::vector<uint8_t>& buffer) {
        uint8_t chksum = 0;
        for (auto& _byte : buffer) {
            chksum += _byte;
        }
        return chksum & 0xFF;
    }

    std::vector<uint8_t> SerialMessage::packFloats(const std::vector<float> &values) {

        auto buffer = std::vector<uint8_t>();
        for (auto& value : values) {

            auto bytes = packFloat(value);
            buffer.insert(buffer.end(), bytes.begin(), bytes.end());
        }
        return buffer;
    }

    std::vector<uint8_t> SerialMessage::packFloat(const float& value) {

        auto buffer = std::vector<uint8_t>();

        // Represent as 4-byte integer
        uint32_t intRepresentation;
        std::memcpy(&intRepresentation, &value, sizeof(float));

        // Convert to big endian
        buffer.emplace_back((intRepresentation >> 24) & 0xFF);
        buffer.emplace_back((intRepresentation >> 16) & 0xFF);
        buffer.emplace_back((intRepresentation >> 8) & 0xFF);
        buffer.emplace_back(intRepresentation & 0xFF);

        return buffer;
    }

    std::vector<uint8_t> SerialMessage::serialize() {

        // Add header to the buffer
        std::vector<uint8_t> buffer = header->toBytes();
        buffer.emplace_back(getSequenceByte());

        // Add payload
        std::vector<uint8_t> inputBytes{};
        for (const auto& input : inputs) {
            if (std::holds_alternative<float>(input)) {
                float value = std::get<float>(input);
                //std::cout << "Found float: " << value << std::endl;
                std::vector<uint8_t> packedBytes = packFloat(value);
                inputBytes.insert(inputBytes.end(), packedBytes.begin(), packedBytes.end());
            } else if (std::holds_alternative<uint16_t>(input)) {
                uint16_t value = std::get<uint16_t>(input);
                //std::cout << "Found uint16_t: " << static_cast<int>(value) << std::endl;
                auto byte1 = static_cast<uint8_t>((value >> 8) & 0xFF);
                auto byte2 = static_cast<uint8_t>(value & 0xFF);
                inputBytes.emplace_back(byte1);
                inputBytes.emplace_back(byte2);
            } else {
                uint8_t value = std::get<uint8_t>(input);
                //std::cout << "Found uint8_t: " << static_cast<int>(value) << std::endl;
                inputBytes.emplace_back(value);
            }
        }
        buffer.insert(buffer.end(), inputBytes.begin(), inputBytes.end());
        // Perform and add checksum
        uint8_t chksum = checksum(buffer) ^ 0xFF;
        buffer.emplace_back(chksum);
        //std::cout << "checksum:" << static_cast<int>(chksum) << std::endl;

        // Add start of packet
        buffer.insert(buffer.begin(), START_OF_PACKET);

        // Add end of packet
        buffer.insert(buffer.end(), END_OF_PACKET);

        return buffer;
    }

    std::vector<uint8_t> SerialMessage::unescapeBuffer(std::vector<uint8_t>& buffer) {
        std::vector<uint8_t> outBuf{};

        for (int i = 0; i < buffer.size(); i++) {
            if (buffer[i] == ESCAPE) {
                i++;
                if (buffer[i] == ESCAPED_START) outBuf.emplace_back(START_OF_PACKET);
                else if (buffer[i] == ESCAPED_END) outBuf.emplace_back(END_OF_PACKET);
                else if (buffer[i] == ESCAPED_ESCAPE) outBuf.emplace_back(ESCAPE);
            } else {
                outBuf.emplace_back(buffer[i]);
            }
        }

        return outBuf;
    }

    /*
     Creates a SerialMessage object from a given response bytearray
     */
    std::shared_ptr<SerialMessage> SerialMessage::fromBuffer(std::vector<uint8_t> &buffer) {

        buffer = unescapeBuffer(buffer);
        // Create object
        auto msg = std::make_shared<SerialMessage>();

        uint16_t startIndex = 0;
        uint16_t endIndex = 0;

        try {
            for (uint16_t i = 0; i < buffer.size(); i++) {
                if (buffer[i] == START_OF_PACKET) {
                    startIndex = i;
                    break;
                }
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
        }

        try {
            for (uint16_t i = startIndex; i < buffer.size(); i++) {
                if (buffer[i] == END_OF_PACKET) {
                    endIndex = i;
                }
            }
        } catch (const std::exception &e) {
            std::cerr << e.what() << std::endl;
        }
        // Remove SOP and EOP
        buffer.erase(buffer.begin() + endIndex); // EOP
        buffer.erase(buffer.begin() + startIndex); // SOP

        // Perform checksum
        auto chksum = checksum(buffer);
        if (chksum != 0xFF) {
            std::cerr << "Checksum error" << buffer.size() << std::endl;
        }


        // Create header from data
        msg->header = SerialHeader::fromBuffer(buffer);
        // Create body from remaining buffer
        buffer.erase(buffer.begin(), buffer.begin() + msg->header->byte_length);
        buffer.pop_back();
        msg->body = buffer;

        return msg;
    }

}