//
// Created by peter on 29.10.24.
//
#include "Rover.hpp"
#include "RoverStateStreamer.hpp"

#include <thread>
#include <chrono>
#include <iostream>
#include "lidarDriver.hpp"
#include "communication.hpp"
#include "simple_socket/TCPSocket.hpp"
#include <atomic>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <queue>

void sigpipeHandler(int signal) {
    std::cout << "Client disconnected" << std::endl;
}

int main() {
    signal(SIGPIPE, sigpipeHandler);

    Rover::Rover rover;
    rover.init();

    std::thread roverThread([&rover] {
        rover.startSensorStream(33);
        while (true) {
            if (rover.parser->sensorDataAvailable()) {
                std::mutex m;
                std::unique_lock<std::mutex> lock(m);
                const float* data = rover.getState();
                lock.unlock();

                std::vector<uint8_t> packet{};
                for (int i = 12; i >= 0; i--) {
                    uint8_t temp[sizeof(float)];
                    std::memcpy(temp, &data[i], sizeof(float));
                    packet.insert(packet.begin(), temp, temp + sizeof(float));
                }
                rover.stateStreamer->addPacket(packet);
            }
        }
    });

    std::queue<uint8_t> commands;
    std::thread updateThread([&rover, &commands] {
        float heading = 0.0f;
        float speed = 0.2f;
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
                    heading += speed*0.5f;
                    if (rover.driving) rover.driveWithYaw(speed, heading);
                }
                if (command == 0x03) {
                    heading -= speed*0.5f;
                    if (rover.driving) rover.driveWithYaw(speed, heading);
                }
                if (command == 0x04) {
                    speed = -abs(speed);
                    rover.driveWithYaw(speed, heading);
                }
                if (command == 0x05) {
                    rover.stop();
                }
            }
            rover.update();
        }
    });

    LidarDriver lidar;
    TCPServer server(9998);
    std::atomic<bool> stopper(false);
    auto connection = server.accept();
    std::cout << "Client connected" << std::endl;

    Communication com;
    std::vector<std::pair<double,double>> coordinates;
    std::mutex coordinates_mutex;

    lidar.startMotorHalf();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::queue<std::vector<std::pair<double,double>>> coordinates_queue;

    std::thread scanThread(
        &LidarDriver::startScan1,
        &lidar,
        std::ref(stopper),
        std::ref(coordinates_queue),
        std::ref(coordinates_mutex));

    std::thread senderThread(
        &Communication::sendLidarData,
        &com,
        std::ref(connection),
        std::ref(coordinates_queue),
        std::ref(coordinates_mutex),
        std::ref(stopper));

    std::thread commandReceiverThread([&commands] {
        TCPClientContext commandClient;
        std::string laptopIP("10.24.41.131");
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

    roverThread.detach();
    updateThread.detach();
    scanThread.detach();
    senderThread.detach();
    commandReceiverThread.join();

    std::cout << "Closing" << std::endl;
    rover.closeConnection();
}

