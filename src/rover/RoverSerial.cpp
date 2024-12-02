//
// Created by peter on 29.10.24.
//
#include "RoverSerial.hpp"

namespace Rover {
    RoverSerial::RoverSerial() {
        device = std::make_unique<serial::Serial>("/dev/ttyTHS0", 115200);
        parser = std::make_unique<SerialParser>();
        serial_thread = std::thread([this] {
            loopRW();
        });
        serial_thread.detach();
    }


    void RoverSerial::loopRW() {
        std::cout << "Starting Rover serial thread."<<std::endl;
        while (!connected) {
            connected = device->isOpen();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for serial port to connect.
        }
        std::cout << "Rover connected."<<std::endl;
        device->flush();
        try {
            while (connected) {
                write();
                read();
            }
        } catch (std::exception& e) {
            std::cout << "Serial Error: " << e.what() << std::endl;
        }
        std::cout << "Serial port closed, exiting serial thread." << std::endl;
    }

    bool RoverSerial::isConnected() {
        return device->isOpen();
    }
    /*
     Writes the first message in the command queue to the serial device.
     */
    void RoverSerial::write() {
        if (writeQueue.empty()) return;
        std::vector<uint8_t> command = writeQueue.front();
        try {
            device->write(command);
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            connected = device->isOpen();
            return;
        }
        writeQueue.pop();
    }

    /*
     Reads available data in the input buffer and parses the data.
     */
    void RoverSerial::read() {
        uint16_t bytesAvailable = device->available();
        if (bytesAvailable > 0) {
            std::vector<uint8_t> data(bytesAvailable); // Create vector with size of bytesAvailable
            try {
                device->read(data, bytesAvailable); // Fill buffer
                if (new_data_handler) {
                    new_data_handler(data);
                }
            } catch (std::exception& e) {
                std::cout << e.what() << std::endl;
                connected = device->isOpen();
            }
        }
    }

    void RoverSerial::addCommand(const std::vector<uint8_t> &command) {
        writeQueue.push(command);
    }

    void RoverSerial::sendCommand(const std::string &key, const std::vector<VariantType> &inputs, const std::vector<VariantType> &outputs) {
        SerialMessage message;

        message.header = serialCommands[key];
        message.inputs = inputs;
        message.outputs = outputs;

        auto command = message.serialize();
        addCommand(command);
    }


    void RoverSerial::open() {
        device->open();
        serial_thread = std::thread([this] { loopRW(); });
        serial_thread.detach();
    }

    void RoverSerial::close() {
        device->close();
        connected = false;
    }

    std::shared_ptr<SerialMessage> RoverSerial::collectResponse() {
        auto msg = responseQueue.front();
        responseQueue.pop();
        return msg;
    }
}