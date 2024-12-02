//
// Created by peter on 31.10.24.
//

#ifndef ROVERSERIALSENSORSTREAMSERVICE_HPP
#define ROVERSERIALSENSORSTREAMSERVICE_HPP
#include "RoverSerialSensorStream.hpp"
#include "SerialEnum.hpp"

namespace Rover {

    enum StreamingDataSizes {
        EIGHT_BITS = 0x00,
        SIXTEEN_BITS = 0x01,
        THIRTYTWO_BITS = 0x02
    };

    class SensorStreamService {
    public:
        uint8_t id;
        std::string name;
        uint8_t dataSize;
        uint8_t byteCount;
        std::vector<SensorAttribute> attributes;
        uint8_t processor;

        SensorStreamService(const uint8_t& id, const std::string& name, const uint8_t& dataSize, const std::vector<SensorAttribute>& attributes, const uint8_t& processor) :
        id(id), name(name), dataSize(dataSize), byteCount(std::pow(2, dataSize)), attributes(attributes), processor(processor) {};

        std::vector<float> parseAttributeBytesToFloats(std::vector<uint8_t> &bytes);
        std::vector<int> parseAttributeBytesToInts(std::vector<uint8_t> &bytes);
        std::vector<uint32_t> getValuesFromBytes(std::vector<uint8_t> &bytes);

        float normalize(float value, float in_min, float in_max, float out_min, float out_max);
        int normalize(uint32_t value, uint32_t in_min, uint32_t in_max, int out_min, int out_max);
    };
};
#endif //ROVERSERIALSENSORSTREAMSERVICE_HPP
