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

void LidarDriver::start() {
    if (!scanning) {
        startMotorHalf();
        startExpressScan();
    }
}

void LidarDriver::stop() {
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

void LidarDriver::startScan() {
    std::thread scan_thread([this] {
        scanning = true;
        device->flush();

        //Send the startScan command
        std::vector<unsigned char> startScan = {0xA5, 0x20};

        std::vector<uint8_t> startScan_bytes = checksum(startScan);
        device->write(startScan_bytes);

        //std::cout<<"Sending startScan"<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read the first packet from LiDAR
        std::vector<uint8_t> inputData;
        //std::cout << "Available: " << device->available() << std::endl;
        while (device->available() < 7) {
            //std::cout << "Available: " << device->available() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        device->read(inputData,7);

        // Ensure the vector has enough data to prevent out-of-range access, debugging stuff

        /*
        std::cout << "Received inputData size: " << inputData.size() << std::endl;
        for (const auto &byte : inputData) {
            std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << std::dec << std::endl;*/

        if (inputData.size() < 7) {
            throw std::runtime_error("Failed to read initial packet from LiDAR: Insufficient data");
        }

        auto start1 = inputData[0];
        auto start2 = inputData[1];
        auto packetsize = inputData[2];
        auto midpackets = (inputData[3] << 4) + (inputData[4] << 2) + inputData[5];
        auto ending = inputData[6];

        bool validStartPacket = (   start1 ==165 &&
                                    start2==90 &&
                                    packetsize==5 &&
                                    midpackets==64 &&
                                    ending==129);

        if (!validStartPacket) return;

        int val = 0;
        int quality = 0;
        float angleLowByte = 0;
        float angleHighByte = 0;
        float distHighbyte=0;
        float distLowbyte=0;
        float angle = 0;
        float dist=0;
        auto DEG90 = 3.14159265f/2.0f;
        float DEG2RAD = 3.14159265f/180.0f;
        float DISTANCE_SCALING = 1/4000.0f;
        float ANGLE_SCALING = 1/64.0f;


        std::vector<std::pair<float, float>> coordinates;
        std::vector<uint8_t> SOF_ = {0xd8, 0xd9, 0xda};

        while (scanning) {

            int available = device->available();
            if (available < 5) continue;
            std::vector<uint8_t> packet;
            int bytesPerPoint = 5;
            int bytesToRead = available - available % bytesPerPoint;
            device->read(packet,bytesToRead);
            for (int i = 0; i < packet.size() / bytesPerPoint; i++) {
                int offset = i*bytesPerPoint;

                val = (packet[offset]) & 3;
                quality = (packet[offset] >> 2);
                if (val == 1) {
                    std::mutex m;
                    uint16_t points = coordinates.size();
                    std::unique_lock<std::mutex> lock(m);
                    std::vector<uint8_t> data;
                    serializeLidarData(coordinates, data);
                    coordinates.clear(); //Clears the coordinates so that it starts on a new scan frame
                    lock.unlock();

                    // Insert amount of points in frame
                    data.insert(data.begin(), points);

                    if (new_frame_handler) new_frame_handler(data);
                }
                angleLowByte = packet[1 + offset] >> 1;
                angleHighByte = packet[2 + offset] << 7;

                distLowbyte = packet[3 + offset];
                distHighbyte = packet[4 + offset] << 8;

                dist = (distHighbyte + distLowbyte) * DISTANCE_SCALING;
                if (dist > 0) {
                    angle = (angleHighByte + angleLowByte) * ANGLE_SCALING * DEG2RAD;
                    auto x = std::sin(angle + DEG90) * dist;
                    auto y = std::cos(angle + DEG90) * dist;
                    coordinates.emplace_back(x,y);
                }
            }
        }
    });

    scan_thread.detach();
}

void LidarDriver::startExpressScan() {
    std::thread scan_thread([this] {
        scanning = true;
        device->flush();

        std::vector<unsigned char> startScan = {0xA5, 0x82,0x05,0x00,0x00,0x00,0x00,0x00};

        std::vector<uint8_t> startScan_bytes = checksum(startScan);
        device->write(startScan_bytes);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::vector<uint8_t> inputData;
        while (device->available() < 7) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        device->read(inputData,7);

        if (inputData.size() < 7) {
            throw std::runtime_error("Failed to read initial packet from LiDAR: Insufficient data");
        }

        auto start1 = inputData[0];
        auto start2 = inputData[1];
        auto packetsize = inputData[2];
        auto midpackets = (inputData[3] << 4) + (inputData[4] << 2) + inputData[5];
        auto ending = inputData[6];

        bool validStartPacket = (   start1 ==165 &&
                                    start2==90 &&
                                    packetsize==84 &&
                                    midpackets==64 &&
                                    ending==130);

        if (!validStartPacket) return;
        std::cout << "VALID LIDAR PACKET" << std::endl;
        uint8_t byteNum = 0;
        uint16_t refAngleP1 = 0;
        uint16_t refAngleP2 = 0;
        float refAngle = 0;
        uint16_t delta1P1 = 0;
        uint16_t delta1P2 = 0;
        std::vector<float> delta1;
        int deltaSign1 = 1;
        uint16_t delta2P1 = 0;
        uint16_t delta2P2 = 0;
        std::vector<float> delta2;
        uint16_t dist1P1 = 0;
        uint16_t dist1P2 = 0;
        int deltaSign2 = 1;
        std::vector<float> dist1;
        uint16_t dist2P1 = 0;
        uint16_t dist2P2 = 0;
        std::vector<float> dist2;
        float previousAngle = 0;
        auto DEG90 = 3.14159265f/2.0f;
        float DEG2RAD = 3.14159265f/180.0f;
        float DISTANCE_SCALING = 1/4000.0f;
        float ANGLE_SCALING = 1/64.0f;
        uint32_t scanCount = 0;
        std::vector<uint8_t> SOF_ = {0xd8, 0xd9, 0xda};

        std::vector<std::pair<float, float>> points;
        while (scanning) {
            size_t available = device->available();
            if (available == 0) continue;
            int byteCount = 0;
            std::vector<uint8_t> packet;
            device->read(packet,available);
            while (byteCount < available) {
                if (byteNum == 0) {
                    uint8_t check1 = (packet[byteCount] >> 4);
                    if (check1 == 10) {
                        byteNum = 1;
                    }
                } else if ( byteNum == 1 ) {
                    uint8_t check2 = (packet[byteCount] >> 4);
                    check2 == 5 ? byteNum = 2 : byteNum = 0;
                    
                } else if ( byteNum == 2 ) {
                        byteNum = 3;
                        refAngleP2 = packet[byteCount];
                } else if ( byteNum == 3 ) {
                        refAngleP1 = ((packet[byteCount] & 127) << 8);
                        refAngle = (static_cast<float>(refAngleP1 + refAngleP2) / 64.0f) * DEG2RAD;

                        if (refAngle - previousAngle < 0 ) {
                            scanCount += 1;
                            std::vector<uint8_t> data;
                            serializeLidarData(points, data);

                            // Insert amount of points in frame
                            uint16_t dataPoints = points.size();

                            uint8_t dPoints1 = dataPoints >> 8;
                            uint8_t dPoints2 = dataPoints & 0xff;
                            data.insert(data.begin(), dPoints2);
                            data.insert(data.begin(), dPoints1);
                            if (new_frame_handler) new_frame_handler(data);
                            points.clear();
                        }

                        if (scanCount > 1 ) {
                            calcAndAddPoints(points, dist1,dist2,delta1,delta2,previousAngle,refAngle);
                            dist1.clear();
                            dist2.clear();
                            delta1.clear();
                            delta2.clear();
                        }
                        byteNum = 4;
                        
                } else if ( byteNum == 4 ) {
                    byteNum = 5;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 5 ) {
                    byteNum = 6;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 6 ) {
                    byteNum = 7;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 7 ) {
                    byteNum = 8;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 8 ) {
                    byteNum = 9;
                    std::tie(delta1P1, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 9 ) {
                    byteNum = 10;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 10 ) {
                    byteNum = 11;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 11 ) {
                    byteNum = 12;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 12 ) {
                    byteNum = 13;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 13 ) {
                    byteNum = 14;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 14 ) {
                    byteNum = 15;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 15 ) {
                    byteNum = 16;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 16 ) {
                    byteNum = 17;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 17 ) {
                    byteNum = 18;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 18 ) {
                    byteNum = 19;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 19 ) {
                    byteNum = 20;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 20 ) {
                    byteNum = 21;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 21 ) {
                    byteNum = 22;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 22 ) {
                    byteNum = 23;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 23 ) {
                    byteNum = 24;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 24 ) {
                    byteNum = 25;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 25 ) {
                    byteNum = 26;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 26 ) {
                    byteNum = 27;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 27 ) {
                    byteNum = 28;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 28 ) {
                    byteNum = 29;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 29 ) {
                    byteNum = 30;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 30 ) {
                    byteNum = 31;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 31 ) {
                    byteNum = 32;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 32 ) {
                    byteNum = 33;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 33 ) {
                    byteNum = 34;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 34 ) {
                    byteNum = 35;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 35 ) {
                    byteNum = 36;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 36 ) {
                    byteNum = 37;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 37 ) {
                    byteNum = 38;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 38 ) {
                    byteNum = 39;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 39 ) {
                    byteNum = 40;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 40 ) {
                    byteNum = 41;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 41 ) {
                    byteNum = 42;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 42 ) {
                    byteNum = 43;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 43 ) {
                    byteNum = 44;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 44 ) {
                    byteNum = 45;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 45 ) {
                    byteNum = 46;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 46 ) {
                    byteNum = 47;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 47 ) {
                    byteNum = 48;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 48 ) {
                    byteNum = 49;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 49 ) {
                    byteNum = 50;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 50 ) {
                    byteNum = 51;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 51 ) {
                    byteNum = 52;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 52 ) {
                    byteNum = 53;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 53 ) {
                    byteNum = 54;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 54 ) {
                    byteNum = 55;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 55 ) {
                    byteNum = 56;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 56 ) {
                    byteNum = 57;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 57 ) {
                    byteNum = 58;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 58 ) {
                    byteNum = 59;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 59 ) {
                    byteNum = 60;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 60 ) {
                    byteNum = 61;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 61 ) {
                    byteNum = 62;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 62 ) {
                    byteNum = 63;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 63 ) {
                    byteNum = 64;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 64 ) {
                    byteNum = 65;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 65 ) {
                    byteNum = 66;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 66 ) {
                    byteNum = 67;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 67 ) {
                    byteNum = 68;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 68 ) {
                    byteNum = 69;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 69 ) {
                    byteNum = 70;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 70 ) {
                    byteNum = 71;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 71 ) {
                    byteNum = 72;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 72 ) {
                    byteNum = 73;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 73 ) {
                    byteNum = 74;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 74 ) {
                    byteNum = 75;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 75 ) {
                    byteNum = 76;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 76 ) {
                    byteNum = 77;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 77 ) {
                    byteNum = 78;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 78 ) {
                    byteNum = 79;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                } else if ( byteNum == 79 ) {
                    byteNum = 80;
                    std::tie(dist1P2, delta1P1, deltaSign1) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 80 ) {
                    byteNum = 81;
                    dist1P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 81 ) {
                    byteNum = 82;
                    std::tie(dist2P2, delta2P1, deltaSign2) = readCabin1or3(packet[byteCount]);
                } else if ( byteNum == 82 ) {
                    byteNum = 83;
                    dist2P1 = readCabin2or4(packet[byteCount]);
                } else if ( byteNum == 83 ) {
                    byteNum = 0;
                    std::tie(delta1P2, delta2P2) = readCabin5(packet[byteCount]);
                    dist1.push_back(calcDist(dist1P1, dist1P2));
                    dist2.push_back(calcDist(dist2P1, dist2P2));
                    delta1.push_back(calcDelta(delta1P1, delta1P2, deltaSign1));
                    delta2.push_back(calcDelta(delta2P1, delta2P2, deltaSign2));
                    previousAngle = refAngle;
                }
                byteCount += 1;
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

void LidarDriver::serializeLidarData(const std::vector<std::pair<float, float>>& data, std::vector<uint8_t>& buffer) {
    int index = 0;
    for (const auto& pair : data) {
        std::vector<uint8_t> bytes(sizeof(pair));
        std::memcpy(bytes.data(), &pair, sizeof(pair)); // Copy each float into bytes
        buffer.insert(buffer.begin() + index, bytes.begin(), bytes.end());
        index += sizeof(pair);
    }
}

void LidarDriver::calcAndAddPoints(std::vector<std::pair<float, float>>& points, const std::vector<float>& dist1, const std::vector<float>& dist2, const std::vector<float>& delta1, const std::vector<float>& delta2, float previousAngle, float refAngle) {
    float angleDiff = refAngle - previousAngle;
    if (angleDiff < 0) {
        angleDiff += (3.14159265 * 2.0f);
    }

    const float deg90 = 3.14159265 / 2.0f;
    float ANGLE_OFFSET = -9.0f*3.14159265/180.0f;
    int pointCount = -1;
    for (int i = 0; i < 16; ++i) {
        pointCount++;
        if (dist1[i] > 0) {
            float ptAngle1 = previousAngle + ((angleDiff / 32.0f) * static_cast<float>(pointCount)) - delta1[i];
            float x = std::round(std::sin(ptAngle1 + deg90 + ANGLE_OFFSET) * dist1[i] * 1000.0f) / 1000.0f;
            float y = std::round(std::cos(ptAngle1 + deg90 + ANGLE_OFFSET) * dist1[i] * 1000.0f) / 1000.0f;
            points.push_back({x, y});
        }
        pointCount++;
        if (dist2[i] > 0) {
            float ptAngle2 = previousAngle + ((angleDiff / 32.0f) * static_cast<float>(pointCount)) - delta2[i];
            float x = roundf(std::sin(ptAngle2 + deg90 + ANGLE_OFFSET) * dist2[i] * 1000.0f) / 1000.0f;
            float y = roundf(std::cos(ptAngle2 + deg90 + ANGLE_OFFSET) * dist2[i] * 1000.0f) / 1000.0f;
            points.push_back({x, y});
        }
    }
}
std::tuple<uint16_t, uint16_t, int> LidarDriver::readCabin1or3(uint8_t data) {
    uint16_t distP2 = (data >> 2);
    uint16_t deltaP1 = ((data & 1) << 4);
    int deltaSign = -1 * ((data >> 1) & 1);
    if (deltaSign == 0) {
        deltaSign = 1;
    }
    return {distP2, deltaP1, deltaSign};
}

uint16_t LidarDriver::readCabin2or4(uint8_t data) {
    return (data << 6);
}

std::pair<uint16_t, uint16_t> LidarDriver::readCabin5(uint8_t data) {
    uint16_t delta2P2 = (data >> 4);
    uint16_t delta1P2 = (data & 15);
    return {delta1P2, delta2P2};
}

float LidarDriver::calcDist(uint16_t distP1, uint16_t distP2) {
    return static_cast<float>(distP1 + distP2) / 1000.0f; // distance in meters
}

float LidarDriver::calcDelta(uint16_t deltaP1, uint16_t deltaP2, int deltaSign) {
    float value = static_cast<float>(deltaSign) * (static_cast<float>(deltaP1 + deltaP2) / 8.0f) * (3.141592 / 180.0f);
    return value; // delta angle in radians
}