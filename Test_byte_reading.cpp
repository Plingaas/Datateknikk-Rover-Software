#include "Lidar.hpp"
#include "Rover.hpp"
#include "signal.h"
int main() {
    signal(SIGPIPE, SIG_IGN); // Avoid socket->write closing program. Handlers take care of errors.
    signal(SIGABRT, SIG_IGN);

    Rover::Rover rover(9996);
    rover.init();
    rover.setServerTimeout(5000);
    rover.startTCPServer();

    uint16_t lidarTCPPort = 9998;
    Lidar lidar(lidarTCPPort);
    lidar.init();
    lidar.setServerTimeout(5000);
    lidar.startTCPServer();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}