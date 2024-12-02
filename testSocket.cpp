//
// Created by peter on 04.11.24.
//
#include "Lidar.hpp"
#include "Rover.hpp"
#include "CameraStreamer.hpp"
#include <signal.h>
#include <thread>
#include <chrono>
int main() {

    CameraStreamer cameraStreamer(1, 640, 480, 60);
    cameraStreamer.setClient("10.24.41.131", 8553);
    cameraStreamer.start();

    uint16_t roverTCPPort = 9996;
    Rover::Rover rover(roverTCPPort);
    rover.init();
    rover.startSerialSensorStream(33);
    rover.setServerTimeout(5000);
    rover.startTCPServer();

    uint16_t lidarTCPPort = 9998;
    Lidar lidar(lidarTCPPort);
    lidar.init();
    lidar.setServerTimeout(5000);
    lidar.startTCPServer();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
