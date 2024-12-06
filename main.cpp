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
//#include "CameraStreamer.hpp"





int main(int argc, char** argv) {
    signal(SIGPIPE, SIG_IGN);

    CameraStreamer cameraStreamer(4, 416, 416, 30);
    cameraStreamer.setClient("10.24.41.131", 8553);
    cameraStreamer.start();

    uint16_t roverTCPPort = 9996;
    if (argc > 2) roverTCPPort = std::atoi(argv[2]);
    Rover::Rover rover(roverTCPPort);
    rover.startSerialSensorStream(33);
    rover.setServerTimeout(5000);
    rover.startTCPServer();

    uint16_t lidarTCPPort = 9998;
    if (argc > 3) lidarTCPPort = std::atoi(argv[3]);
    Lidar lidar(lidarTCPPort);
    lidar.init();
    lidar.setServerTimeout(5000);
    lidar.startTCPServer();

    std::queue<std::vector<uint8_t>> commands;
    std::mutex m;
    float speed = 0.3;
    if (argc > 1) speed = std::atof(argv[1]);
    std::thread updateThread([&rover, &commands, &m, &speed] {
        float heading = 0.0f;
        while (true) {
            if (!commands.empty()) {
                std::unique_lock<std::mutex> lock(m);
                auto commandBuffer = commands.front();
                uint8_t command = commandBuffer[0];
                commands.pop();
                lock.unlock();

                if (command == 0x05) {
                    rover.stopDriving();
                }
                if (command == 0x01) {
                    speed = abs(speed);
                    //rover.driveWithYaw(speed, heading);
                    rover.driveTank(0.2, 0.2);
                }
                if (command == 0x02) {
                    //heading += speed*2;
                    //if (rover.driving) rover.driveWithYaw(speed, heading);
                    rover.driveTank(-0.2, 0.2);
                }
                if (command == 0x03) {
                    //heading -= speed*2;
                    //if (rover.driving) rover.driveWithYaw(speed, heading);
                    rover.driveTank(0.2, -0.2);
                }
                if (command == 0x04) {
                    speed = -abs(speed);
                    //rover.driveWithYaw(speed, heading);
                    rover.driveTank(-0.2, -0.2);
                }
                if (command == 0x06) {
                    speed = abs(speed);
                    float targetHeading;
                    std::memcpy(&targetHeading, commandBuffer.data()+1, sizeof(float));
                    std::cout << speed << " | " << targetHeading << std::endl;
                    rover.driveWithYaw(speed, targetHeading);
                }

            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });


    std::thread commandReceiverThread([&commands, &m] {
        TCPClientContext commandClient;

        while (true) {
            auto commandConnection = commandClient.connect("10.24.41.131", 9876);
            std::cout << "Connected to command server. Connected = "<<std::endl;
	    if (!commandConnection){
	        this_thread::sleep_for(chrono::milliseconds(100));
	        continue;
	    };

            float heading = 0.0f;
            while (true) {
                std::vector<uint8_t> buffer(16);
                int bytes = commandConnection->read(buffer);
                if (bytes == -1) break;
                std::cout << "Got command" << std::endl;
                if (!buffer.empty()) {
                    std::lock_guard<std::mutex> lock(m);
                    commands.push(buffer);
                }
            }
        }

    });

    updateThread.detach();
    commandReceiverThread.join();

}

