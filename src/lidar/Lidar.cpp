//
// Created by peter on 01.12.24.
//
#include "Lidar.hpp"

Lidar::Lidar(uint16_t TCPPort) {
    driver = std::make_unique<LidarDriver>();
    // Show that rover is ready to connect
    driver->startMotorHalf();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    driver->stop();

    streamer = std::make_unique<TCPStreamer>(TCPPort, true);
    streamer->setName("Lidar");

    // Serial handlers
    driver->setConnectHandler([this] {streamer->startStreaming();});
    driver->setCloseHandler([this] {streamer->close();});
    driver->setNewFrameHandler([this](std::vector<uint8_t>& data) {streamer->addPacket(data);});

    // Server handlers
    streamer->setDisconnectHandler([this]() {this->onClientDisconnect();});
    streamer->setConnectHandler([this]() {this->onClientConnect();});
    streamer->setReconnectHandler([this] {this->onClientReconnect();});

}

void Lidar::init() {
    driver->stop();
}

void Lidar::onClientConnect() {
    driver->start();
}
void Lidar::onClientDisconnect() {
    driver->stop();
    clearScanFrames();
}
void Lidar::onClientReconnect() {
}

void Lidar::clearScanFrames() {
    while (!scan_frames.empty()) {
        scan_frames.pop();
    }
}


