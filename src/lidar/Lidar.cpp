//
// Created by peter on 01.12.24.
//
#include "Lidar.hpp"

Lidar::Lidar() {
    driver = std::make_unique<LidarDriver>();
    streamer = std::make_unique<TCPStreamer>(9998, true);

    streamer->setTimeout(1000);

    driver->setConnectHandler([this] {
        streamer->startStreaming();
    });

    driver->setCloseHandler([this] {
        streamer->close();
    });

    driver->setNewFrameHandler([this](std::vector<uint8_t>& data) {
        streamer->addPacket(data);
    });

    streamer->setDisconnectHandler([this]() {
       this->onClientDisconnect();
    });

    streamer->setConnectHandler([this]() {
        this->onClientConnect();
    });

    streamer->setReconnectHandler([this] {
        this->onClientReconnect();
    });

    //std::vector<uint8_t> packet = { 0x01, 0x02, 0x03 };
    //streamer->addPacket(packet);
    streamer->startStreaming();
}

void Lidar::onClientConnect() {
    std::cout << "Lidar client connected." << std::endl;
    driver->startMotorHalf();
    driver->startScan(&scan_frames);
}
void Lidar::onClientDisconnect() {
    std::cout << "Lidar client disconnected." << std::endl;
    driver->stopScan();
    driver->stopMotor();
    clearScanFrames();
}
void Lidar::onClientReconnect() {
    std::cout << "Lidar client reconnected." << std::endl;
}

void Lidar::clearScanFrames() {
    while (!scan_frames.empty()) {
        scan_frames.pop();
    }
}
