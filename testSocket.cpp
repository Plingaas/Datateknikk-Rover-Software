//
// Created by peter on 04.11.24.
//
#include "RoverStateStreamer.hpp"
#include <thread>
#include <chrono>
int main() {

  Rover::StateStreamer streamer;

  std::vector<uint8_t> data = {'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd'};
  streamer.addPacket(data);

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

}
