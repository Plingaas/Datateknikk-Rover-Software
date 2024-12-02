//
// Created by peter on 01.12.24.
//
#include "Lidar.hpp"

Lidar::Lidar(uint16_t TCPPort) {
    driver = std::make_unique<LidarDriver>();
    streamer = std::make_unique<TCPStreamer>(TCPPort, true);
    streamer->setName("Lidar");

    // Serial handlers
    driver->setConnectHandler([this] {
        streamer->startStreaming();
    });

    driver->setCloseHandler([this] {
        streamer->close();
    });

    driver->setNewFrameHandler([this](std::vector<uint8_t>& data) {
        streamer->addPacket(data);
    });

    // Server handlers
    streamer->setDisconnectHandler([this]() {
       this->onClientDisconnect();
    });

    streamer->setConnectHandler([this]() {
        this->onClientConnect();
    });

    streamer->setReconnectHandler([this] {
        this->onClientReconnect();
    });

}

void Lidar::init() {
    driver->stopScan();
    driver->stopMotor();
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
