//
// Created by peter on 29.10.24.
//

#ifndef ROVERSERIAL_HPP
#define ROVERSERIAL_HPP
#include "SerialEnum.hpp"
#include "RoverSerialMessage.hpp"
#include "RoverSerialParser.hpp"
#include "serial/serial.h"
#include <queue>
#include <cstdint>
#include <thread>
#include <utility>
#include <variant>
#include <atomic>

namespace Rover {
    class RoverSerial {
    private:
        const uint8_t START_OF_PACKET = 0x8d;
        const uint8_t END_OF_PACKET = 0xd8;
        void addCommand(const std::vector<uint8_t>& command);

        std::unique_ptr<serial::Serial> device;
        std::thread serial_thread;
        std::atomic<bool> connected;
        std::queue<std::vector<uint8_t>> writeQueue;
        std::queue<std::shared_ptr<SerialMessage>> responseQueue;
        std::unique_ptr<SerialParser> parser;

        void write();
        void read();
        void loopRW();

    public:
        explicit RoverSerial();

        void open();
        bool isConnected();
        void close();
        uint16_t available() { return device->available();};
        [[nodiscard]] std::shared_ptr<SerialMessage> collectResponse();
        bool hasUnreadResponse() {return !responseQueue.empty();};
        void sendCommand(const std::string& key,
            const std::vector<VariantType> &inputs = std::vector<VariantType>(),
            const std::vector<VariantType> &outputs = std::vector<VariantType>());

    };
}
#endif //ROVERSERIAL_HPP
