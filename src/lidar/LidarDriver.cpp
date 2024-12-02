#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include "LidarDriver.hpp"

LidarDriver::LidarDriver() {

    // Initialize serial device
    device = std::make_unique<serial::Serial>("/dev/ttyUSB0", 256000);
    device->flush();
    device->setDTR(false);
    if (device->isOpen()) {
        std::cout << "Serial connected successfully" << std::endl;
    }
    stopScan();
    stopMotor();
}

const std::vector<uint8_t> LidarDriver::checksum(const std::vector<unsigned char>& bytes) {
    unsigned char checksum = 0;  // Initialize checksum
    std::vector<uint8_t> newCommand(bytes.begin(), bytes.end());  // Copy input bytes

    // Loop through each byte to compute the checksum (XOR of all bytes)
    for (const auto& el : bytes) {
        checksum ^= el;
    }

    // Append the checksum to the end of the byte vector
    newCommand.push_back(checksum);

    return newCommand;  // Return the vector with the checksum appended
}


void LidarDriver::stopMotor() {

    // Stop command bytes (before checksum)
    std::vector<unsigned char> stop = {0xA5, 0xF0, 0x02, 0x00, 0x00};
    // Add checksum and get the final byte array
    std::vector<uint8_t> stop_bytes = checksum(stop);
    // Write bytes directly to the serial port
    device->write(stop_bytes);

}

void LidarDriver::startMotorHalf() {

    // Start command bytes (before checksum)
    std::vector<unsigned char> start = {0xA5, 0xF0, 0x02, 0x3F, 0x02};
    // Add checksum and get the final byte array
    std::vector<uint8_t> start_bytes = checksum(start);
    // Write bytes directly to the serial port
    device->write(start_bytes);

}

void LidarDriver::startScan(std::queue<std::vector<std::pair<double,double>>>* coordinates_queue) {
    scanning = true;
    std::thread scan_thread([this] {
        device->flush();

        //Send the startScan command
        std::vector<unsigned char> startScan = {0xA5, 0x20};

        std::vector<uint8_t> startScan_bytes = checksum(startScan);
        device->write(startScan_bytes);

        std::cout<<"Sending startScan"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read the first packet from LiDAR
        std::vector<uint8_t> inputData;
        std::cout << "Available: " << device->available() << std::endl;
        while (device->available() < 7) {
            std::cout << "Available: " << device->available() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        device->read(inputData,7);

        // Ensure the vector has enough data to prevent out-of-range access, debugging stuff

        std::cout << "Received inputData size: " << inputData.size() << std::endl;
        for (const auto &byte : inputData) {
            std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;



        if (inputData.size() < 7) {
            throw std::runtime_error("Failed to read initial packet from LiDAR: Insufficient data");
        }

        auto start1 = inputData[0];
        auto start2 = inputData[1];
        auto packetsize = inputData[2];
        auto midpackets = (inputData[3] << 4) + (inputData[4] << 2) + inputData[5];
        auto ending = inputData[6];

        if (start1 ==165 and start2==90 and packetsize==5 and midpackets==64 and ending==129) {

            uint16_t byteNum = 0;
            int val = 0;
            int quality = 0;
            double angleP1 = 0;
            double angleP2 = 0;
            double angle = 0;
            float distP1=0;
            float distP2=0;
            float dist=0;
            auto oldTime = std::chrono::high_resolution_clock::now();
            auto newTime = oldTime;
            auto deg90 = 3.14159265f/2.0f;
            std::vector<std::pair<double, double>> coordinates;

            while (scanning) {

                int available = device->available();
                if (available < 5) continue;
                std::vector<uint8_t> packet(available);
                device->read(packet,available);
                for (int i = 0; i < packet.size(); i++) {

                    if (byteNum == 0) {
                        byteNum = 1;
                        val = (packet[i]) & 3;
                        quality = (packet[i] >> 2);
                        if (val == 1) {
                            std::mutex m;
                            std::unique_lock<std::mutex> lock(m);
                            auto data = serializeLidarData(coordinates);
                            if (new_frame_handler) {
                                new_frame_handler(data);
                            }
                            coordinates.clear(); //Clears the coordinates so that it starts on a new scan frame
                        }
                    }

                    else if (byteNum == 1) {
                        byteNum = 2;
                        angleP2 = packet[i] >> 1;
                    }

                    else if (byteNum == 2) {
                        byteNum = 3;
                        angleP1 = packet[i] << 7;
                        angle = ((angleP1 + angleP2) / 64.0f) * (3.14159265f / 180.0f);
                    }

                    else if (byteNum == 3) {
                        byteNum = 4;
                        distP2 = packet[i];
                    }

                    else if (byteNum == 4) {
                        byteNum = 0;
                        distP1 = packet[i] << 8;
                        dist = (distP1 + distP2) / 4000.0f;
                        if (dist > 0) {
                            auto x = std::sin(angle + deg90) * dist;
                            auto y = std::cos(angle + deg90) * dist;
                            coordinates.emplace_back(x,y);
                        }
                    }
                }
            }
        }
    });

    scan_thread.detach();
}

void LidarDriver::stopScan() {

    // Stop command bytes (before checksum)
    std::vector<uint8_t> stop = {0xA5, 0x25};
    // Add checksum and get the final byte array
    auto stop_bytes = checksum(stop);
    // Write bytes directly to the serial port
    device->write(stop_bytes);
    scanning = false;
}

void LidarDriver::startMotorFull() {

    // Start command bytes (before checksum)
    std::vector<uint8_t> start = {0xA5, 0xF0, 0x02, 0xFF, 0x03};
    // Add checksum and get the final byte array
    auto start_bytes = checksum(start);
    // Write bytes directly to the serial port
    device->write(start_bytes);

}

void LidarDriver::reset() {
    std::vector<uint8_t> Reset= {0xA5, 0x40};
    std::vector<uint8_t> reset_bytes = checksum(Reset);

    device->write(reset_bytes);
}

void LidarDriver::close() {
    device->close();
}

std::vector<uint8_t> LidarDriver::serializeLidarData(const std::vector<std::pair<double, double>>& data) {
    std::vector<uint8_t> buffer(8192);
    //buffer.reserve(data.size() * 2 * sizeof(double)); // Reserve enough space for pairs of doubles
    int index = 0;
    for (const auto& pair : data) {
        std::vector<uint8_t> bytes(sizeof(pair));
        std::memcpy(bytes.data(), &pair, sizeof(pair)); // Copy each double into bytes
        buffer.insert(buffer.begin() + index, bytes.begin(), bytes.end());
        index += sizeof(pair);
    }

    return buffer;
}