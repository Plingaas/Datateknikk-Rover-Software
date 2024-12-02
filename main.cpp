//
// Created by peter on 29.10.24.
//
#include "Rover.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include "Lidar.hpp"
#include "communication.hpp"
#include "simple_socket/TCPSocket.hpp"
#include <queue>
#include "CameraStreamer.hpp"

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

    std::queue<uint8_t> commands;
    std::thread updateThread([&rover, &commands] {
        float heading = 0.0f;
        float speed = 0.4f;
        while (true) {
            if (commands.size() > 0) {
                std::mutex m;
                std::unique_lock<std::mutex> lock(m);
                uint8_t command = commands.front();
                commands.pop();
                lock.unlock();
                std::cout << command << std::endl;
                if (command == 0x01) {
                    speed = abs(speed);
                    rover.driveWithYaw(speed, heading);
                }
                if (command == 0x02) {
                    heading += 3;
                    if (rover.driving) rover.driveWithYaw(speed, heading);
                }
                if (command == 0x03) {
                    heading -= 3;
                    if (rover.driving) rover.driveWithYaw(speed, heading);
                }
                if (command == 0x04) {
                    speed = -abs(speed);
                    rover.driveWithYaw(speed, heading);
                }
                if (command == 0x05) {
                    rover.stopDriving();
                }
            }
        }
    });


    std::thread commandReceiverThread([&commands] {
        TCPClientContext commandClient;
        std::string laptopIP("10.24.41.131");
        std::string hotspotIP("10.42.0.143");
        auto commandConnection = commandClient.connect(laptopIP, 5853);
        std::cout << "Connected to command server." << std::endl;
        float heading = 0.0f;
        while (true) {
            std::vector<uint8_t> buffer(1);
            commandConnection->read(buffer);
            std::mutex m;
            std::unique_lock<std::mutex> lock(m);
            if (buffer[0] != 0x00) {
                commands.push(buffer[0]);
            }
            lock.unlock();
        }
    });

    updateThread.detach();
    commandReceiverThread.join();

}

