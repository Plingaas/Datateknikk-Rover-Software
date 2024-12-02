//
// Created by peter on 30.10.24.
//
#include "RoverSerialParser.hpp"
#include "Rover.hpp"

namespace Rover {
    void printbuffer(std::vector<uint8_t> buffer) {
        std::ostringstream oss;
        for (uint8_t b : buffer) {
            oss << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
        }
        std::cout << oss.str() << std::endl;
    }
    SerialParser::SerialParser() {
        // Initialize buffer
        buffer = std::vector<uint8_t>();

        std::vector<SensorAttribute> IMUAttributes = {
            SensorAttribute("Pitch", -180.0f, 180.0f, FLOAT),
            SensorAttribute("Roll", -90.0f, 90.0f, FLOAT),
            SensorAttribute("Yaw", -180.0f, 180.0f, FLOAT),
        };
        auto IMUService = std::make_shared<SensorStreamService>(0x01, "IMU", 0x02, IMUAttributes, 0x02);
        services.push_back(IMUService);

        std::vector<SensorAttribute> accelerometerAttributes = {
            SensorAttribute("AX", -16.0f, 16.0f, FLOAT),
            SensorAttribute("AY", -16.0f, 16.0f, FLOAT),
            SensorAttribute("AZ", -16.0f, 16.0f, FLOAT),
        };
        auto accelerometerService = std::make_shared<SensorStreamService>(0x02, "Accelerometer", 0x02, accelerometerAttributes, 0x02);
        services.push_back(accelerometerService);

        std::vector<SensorAttribute> gyroscopeAttributes = {
            SensorAttribute("GX", -2000.0f, 2000.0f, FLOAT),
            SensorAttribute("GY", -2000.0f, 2000.0f, FLOAT),
            SensorAttribute("GZ", -2000.0f, 2000.0f, FLOAT),
        };
        auto gyroscopeService = std::make_shared<SensorStreamService>(0x04, "Gyroscope", 0x02, gyroscopeAttributes, 0x02);
        services.push_back(gyroscopeService);

        std::vector<SensorAttribute> locatorAttributes = {
            SensorAttribute("X", -16000.0f, 16000.0f, FLOAT),
            SensorAttribute("Y", -16000.0f, 16000.0f, FLOAT),
        };
        auto locatorService = std::make_shared<SensorStreamService>(0x06, "Locator", 0x02, locatorAttributes, 0x02);
        services.push_back(locatorService);

        std::vector<SensorAttribute> velocityAttributes = {
            SensorAttribute("X", -5.0f, 5.0f, FLOAT),
            SensorAttribute("Y", -5.0f, 5.0f, FLOAT),
        };
        auto velocityService = std::make_shared<SensorStreamService>(0x07, "Velocity", 0x02, velocityAttributes, 0x02);
        services.push_back(velocityService);
    }

    void SerialParser::feed(const std::vector<uint8_t>& data) {
        buffer.insert(buffer.end(), data.begin(), data.end());
    };

    std::shared_ptr<SerialMessage> SerialParser::parse() {
        auto msg = std::make_shared<SerialMessage>();

        // Look for SOP and remove everything before it
        // If no SOP found, clear the buffer and return
        try {
            uint16_t startIndex = 0;
            for (int i = 0; i < buffer.size(); i++) {
                if (buffer[i] == 0x8d) { // Look for SOP byte
                    buffer.erase(buffer.begin(), buffer.begin() + i);
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "SOP not found error: " << e.what() << std::endl;
            buffer.clear();
            return msg;
        }

        // Try to parse a message
        bool skipFutureReads = false;
        try {
            msg = SerialMessage::fromBuffer(buffer);
            skipFutureReads = true;
        } catch (std::exception& e) {
            std::cerr << "Couldnt obtain message from buffer" << std::endl;
        }

        if (!skipFutureReads) {
            parse();
        }

        msg->body.clear();
        //std::cout << "New response: DID: " << static_cast<int>(msg->header->did) << "  CID: " << static_cast<int>(msg->header->cid) << std::endl;
        // Notification for XY position move success or not
        if (msg->header->did == 22 && msg->header->cid == 58) {
            msg->body.insert(msg->body.begin(), buffer.begin(), buffer.begin());
            buffer.erase(buffer.begin());
        }

        // Sensor data
        if (msg->header->did == 24 && msg->header->cid == 61) {
            if (buffer.front() == 0x01 && buffer.size() >= 37) {
                if (buffer.size() < 37) {
                    buffer.clear();
                } else {
                    msg->body.insert(msg->body.begin(), buffer.begin(), buffer.begin() + 37);
                    buffer.erase(buffer.begin(), buffer.begin() + 37);
                }
            }
            else if (buffer.front() == 0x02) {
                if (buffer.size() < 17) {
                    buffer.clear();
                } else {
                    msg->body.insert(msg->body.begin(), buffer.begin(), buffer.begin() + 17);
                    buffer.erase(buffer.begin(), buffer.begin() + 17);
                }
            }
        }

        for (int i = 0; i < buffer.size(); i++) {
            if (buffer[i] == 0xd8) { // Look for EOP byte
                buffer.erase(buffer.begin(), buffer.begin() + i);
                break;
            }
        }
        return msg;
        /*
        std::cout << "DID: " << static_cast<int>(msg->header->did) << std::endl;
        std::cout << "CID: " << static_cast<int>(msg->header->cid) << std::endl;
        std::cout << "TARGET: " << static_cast<int>(msg->header->target) << std::endl;
        std::cout << "SOURCE: " << static_cast<int>(msg->header->source) << std::endl;
        std::cout << "SEQ: " << static_cast<int>(msg->header->seq) << std::endl;
        return msg;
        */


    }

    std::vector<std::vector<float>> SerialParser::parseSensorData1(std::vector<uint8_t>& buffer) {

        std::vector<std::vector<float>> sensorData;
        sensorData.push_back(services[0]->parseAttributeBytesToFloats(buffer));
        sensorData.push_back(services[1]->parseAttributeBytesToFloats(buffer));
        sensorData.push_back(services[2]->parseAttributeBytesToFloats(buffer));
        readToken1 = true;
        return sensorData;
    }

    std::vector<std::vector<float>> SerialParser::parseSensorData2(std::vector<uint8_t>& buffer) {
        std::vector<std::vector<float>> sensorData;
        sensorData.push_back(services[3]->parseAttributeBytesToFloats(buffer));
        sensorData.push_back(services[4]->parseAttributeBytesToFloats(buffer));
        readToken2 = true;
        return sensorData;
    }

    bool SerialParser::sensorDataAvailable() {
        if (readToken1 && readToken2) {
            readToken1 = false;
            readToken2 = false;
            return true;
        }
        return false;
    }
}