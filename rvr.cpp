//
// Created by peter on 29.10.24.
//
#include "Rover.hpp"
#include "RoverStateStreamer.hpp"

#include <thread>
#include <chrono>
#include <iostream>

void printbufffer(std::vector<uint8_t> buffer) {
    std::ostringstream oss;
    for (uint8_t b : buffer) {
        oss << "\\x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    }
    std::cout << oss.str() << std::endl;
}
int main() {
    Rover::Rover rover;

    rover.init();
    std::vector<float> pos = {0.3f, 0.3f};

    rover.driveToPosition(pos, 0.2f, 0.0f, 0);

    while (rover.driving) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    //rover.driveToPosition({0.1f, 0.1f}, 0.2f, 0.0f);
    //rover.stop();
    //rover.driveToPosition({0.0f, 0.2f}, 0.1f, 0.0f);
    //rover.driveRawMotors(128, 128);

}

