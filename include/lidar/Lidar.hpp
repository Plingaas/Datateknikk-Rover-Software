//
// Created by peter on 01.12.24.
//

#ifndef LIDAR_HPP
#define LIDAR_HPP
#include "LidarDriver.hpp"
#include <memory>
#include "TCPStreamer.hpp"
#include <vector>

class Lidar {
private:
    std::unique_ptr<LidarDriver> driver;
    std::unique_ptr<TCPStreamer> streamer;
    std::queue<std::vector<std::pair<double, double>>> scan_frames;

    uint16_t timeout;

    void onClientDisconnect();
    void onClientConnect();
    void onClientReconnect();

    void clearScanFrames();
public:

    Lidar();
};
#endif //LIDAR_HPP
