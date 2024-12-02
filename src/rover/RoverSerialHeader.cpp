//
// Created by peter on 30.10.24.
//

#include "RoverSerialHeader.hpp"

namespace Rover {

    std::shared_ptr<SerialHeader> SerialHeader::fromBuffer(const std::vector<uint8_t>& buffer) {

        uint8_t bitIndex = 0;
        const uint8_t _flags = buffer[bitIndex++];
        bitIndex++;

        auto header = std::make_shared<SerialHeader>(_flags);

        if (header->hasExtendedFlags()) {
            // Shouldn't ever happen so let us not implement it
        }

        if (header->hasTarget()) header->target = buffer[bitIndex++];
        if (header->hasSource()) header->source = buffer[bitIndex++];
        header->did = buffer[bitIndex++];
        header->cid = buffer[bitIndex++];
        header->seq = buffer[bitIndex++];
        if (header->isResponse()) header->err = buffer[bitIndex++];
        header->byte_length = bitIndex;

        return header;
    }
}