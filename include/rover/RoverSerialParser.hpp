//
// Created by peter on 30.10.24.
//

#ifndef ROVERSERIALPARSER_HPP
#define ROVERSERIALPARSER_HPP
#include <cstdint>
#include <vector>
#include <iostream>
#include <RoverSensorData.hpp>
#include "RoverSerialMessage.hpp"
#include "RoverSerialSensorStreamService.hpp"
#include <queue>

namespace Rover {

    class SerialParser {
    private:
        std::vector<std::shared_ptr<SensorStreamService>> services;
        std::mutex bufferMutex;
        bool readToken1 = false;
        bool readToken2 = false;
        std::queue<std::vector<uint8_t>> queue;
        std::vector<uint8_t> buffer{};

    public:
        SerialParser();
        void feed(const std::vector<uint8_t>& data);
        std::shared_ptr<SerialMessage> parse();
        std::vector<std::vector<float>> parseSensorData1(std::vector<uint8_t>& buffer);
        std::vector<std::vector<float>> parseSensorData2(std::vector<uint8_t>& buffer);

        bool sensorDataAvailable();
    };
}
#endif //ROVERSERIALPARSER_HPP
