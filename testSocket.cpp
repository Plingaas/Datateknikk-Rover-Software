//
// Created by peter on 04.11.24.
//
#include "Lidar.hpp"
#include <signal.h>
#include <thread>
#include <chrono>
#include "CameraStreamer.hpp"
int main() {

    CameraStreamer cameraStreamer(1, 640, 480, 60);
    cameraStreamer.setClient("10.24.41.131", 8553);
    cameraStreamer.start();

    Lidar lidar;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
