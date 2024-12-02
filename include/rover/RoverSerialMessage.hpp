//
// Created by peter on 29.10.24.
//

#ifndef ROVERSERIALMESSAGE_HPP
#define ROVERSERIALMESSAGE_HPP

#include "SerialEnum.hpp"
using VariantType = std::variant<float, uint8_t, uint16_t>;

namespace Rover {

    class SerialMessage {
    private:
        static uint8_t sequence;
        static uint8_t getSequenceByte();
        static std::vector<uint8_t> packFloat(const float& value);
        static std::vector<uint8_t> packFloats(const std::vector<float>& values);
        static uint8_t checksum(const std::vector<uint8_t>& buffer);
        static std::vector<uint8_t> unescapeBuffer(std::vector<uint8_t>& buffer);

        inline static uint8_t START_OF_PACKET = 0x8d;
        inline static uint8_t END_OF_PACKET = 0xd8;
        inline static uint8_t ESCAPE = 0xab;
        inline static uint8_t ESCAPED_START = 0x05;
        inline static uint8_t ESCAPED_END = 0x50;
        inline static uint8_t ESCAPED_ESCAPE = 0x23;

        std::string bytes;

    public:
        SerialMessage() = default;
        static std::shared_ptr<SerialMessage> fromBuffer(std::vector<uint8_t>& buffer);
        std::shared_ptr<SerialHeader> header;
        std::vector<uint8_t> body;

        std::vector<VariantType> inputs;
        std::vector<VariantType> outputs;
        std::string getBytes() {return this->bytes;}
        std::vector<uint8_t> serialize();

    };
}
#endif //ROVERSERIALMESSAGE_HPP
