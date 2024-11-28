//
// Created by peter on 31.10.24.
//
#include "RoverSerialSensorStreamService.hpp"

namespace Rover {
    void printbufer(std::vector<uint8_t> buffer) {
        std::ostringstream oss;
        for (uint8_t b : buffer) {
            oss << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
        }
        std::cout << oss.str() << std::endl;
    }
    std::vector<float> SensorStreamService::parseAttributeBytesToFloats(std::vector<uint8_t> &bytes) {

        uint8_t sensorDataIndex = 0;
        std::vector<float> attributeValues;

        for (auto attribute : attributes) {
            std::vector<uint8_t> attributeBytes;

            for (int i = 0; i < byteCount; i++) {
                attributeBytes.push_back(bytes[sensorDataIndex++]);
            }

            std::vector<uint32_t> values = getValuesFromBytes(attributeBytes);

            float normalizedValue = normalize(static_cast<float>(values[0]), 0.0f, static_cast<float>(values[1]), std::get<float>(attribute.minValue), std::get<float>(attribute.maxValue));
            attributeValues.push_back(normalizedValue);
        }

        bytes.erase(bytes.begin(), bytes.begin()+sensorDataIndex);
        return attributeValues;
    };

    std::vector<int> SensorStreamService::parseAttributeBytesToInts(std::vector<uint8_t> &bytes) {
        uint8_t sensorDataIndex = 0;
        std::vector<int> attributeValues;

        for (auto attribute : attributes) {
            std::vector<uint8_t> attributeBytes;

            for (int i = 0; i < byteCount; i++) {
                attributeBytes.push_back(bytes[sensorDataIndex++]);
            }

            std::vector<uint32_t> values = getValuesFromBytes(attributeBytes);
            float normalizedValue = normalize(values[0], 0, values[1], std::get<int>(attribute.minValue), std::get<int>(attribute.maxValue));
            attributeValues.push_back(normalizedValue);
        }
        return attributeValues;
    };

    std::vector<uint32_t> SensorStreamService::getValuesFromBytes(std::vector<uint8_t> &bytes) {
        uint32_t value = 0;
        if (dataSize == EIGHT_BITS) {
          value += static_cast<uint32_t>(bytes[0]);
          return {value, UINT_8_MAX};
        }
        else if (dataSize == SIXTEEN_BITS) {
          value += static_cast<uint32_t>(bytes[0]) << 8;
          value += static_cast<uint32_t>(bytes[1]);
          return {value, UINT_16_MAX};
        }
        else if (dataSize == THIRTYTWO_BITS) {
            value += static_cast<uint32_t>(bytes[0]) << 24;
            value += static_cast<uint32_t>(bytes[1]) << 16;
            value += static_cast<uint32_t>(bytes[2]) << 8;
            value += static_cast<uint32_t>(bytes[3]);
            return {value, UINT_32_MAX};
        }
        else {
          std::cerr << "SensorStreamService: dataSize did not match a valid dataSize. dataSize: " << dataSize << std::endl;
          return {0};
        }
    };

    float SensorStreamService::normalize(float value, float in_min, float in_max, float out_min, float out_max)  {
        return (((value - in_min)/(in_max - in_min)) * (out_max - out_min)) + out_min;
    }
    int SensorStreamService::normalize(uint32_t value, uint32_t in_min, uint32_t in_max, int out_min, int out_max) {
        return std::round((((value - in_min)/(in_max - in_min)) * (out_max - out_min)) + out_min);
    }
};