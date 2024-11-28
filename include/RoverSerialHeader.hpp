//
// Created by peter on 29.10.24.
//

#ifndef ROVERSERIALHEADER_HPP
#define ROVERSERIALHEADER_HPP
#include <vector>
#include <sstream>
#include <memory>

namespace Rover {

    enum Flags {
        PACKET_IS_RESPONSE = 1,
        PACKET_REQUESTS_RESPONSE = 2,
        PACKET_REQUESTS_RESPONSE_IF_ERROR = 4,
        PACKET_IS_ACTIVITY = 8,
        HAS_TARGET = 16,
        HAS_SOURCE = 32,
        PACKET_UNUSED_FLAG_BIT = 64,
        HAS_EXTENDED_FLAGS = 128
    };

    enum ErrorCode {
        SUCCESS,
        BAD_DID,
        BAD_CID,
        NOT_YET_IMPLEMENTED,
        RESTRICTED,
        BAD_DATA_LENGTH,
        FAILED,
        BAD_DATA_VALUE,
        BUSY,
        BAD_TID,
        TARGET_UNAVAILABLE
    };

    class SerialHeader {
    public:
        uint8_t flags;
        uint8_t did;
        uint8_t cid;
        uint8_t seq;
        uint8_t target;
        uint8_t source;
        uint8_t err;
        uint8_t byte_length;

        SerialHeader(   const uint8_t& did = 0x00,
                        const uint8_t& cid = 0x00,
                        const uint8_t& target = 0x00,
                        const uint8_t& source = 0x00,
                        const uint8_t& seq = 0x00,
                        const uint8_t& flags = 0x00,
                        const uint8_t& err = 0x00,
                        const uint8_t& byte_length = 0x00) {
            this->flags = flags;
            this->did = did;
            this->cid = cid;
            this->seq = seq;
            this->target = target;
            this->source = source;
            this->err = err;
            this->byte_length = byte_length;
        };


        [[nodiscard]] bool hasTarget() const {return flags & HAS_TARGET;};
        [[nodiscard]] bool hasSource() const {return flags & HAS_SOURCE;};
        [[nodiscard]] bool hasExtendedFlags() const {return flags & HAS_EXTENDED_FLAGS;};
        [[nodiscard]] bool isResponse() const {return flags & PACKET_IS_RESPONSE;};

        static std::shared_ptr<SerialHeader> fromBuffer(const std::vector<uint8_t>& buffer);

        std::vector<uint8_t> toBytes() {
            return std::vector<uint8_t>({source, target, did, cid});
        }
    };
}
#endif //ROVERSERIALHEADER_HPP
