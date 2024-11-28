#include "lidarDriver.hpp"
#include "communication.hpp"
#include "simple_socket/TCPSocket.hpp"
#include <atomic>
#include <condition_variable>
#include <thread>
#include <iostream>
#include <queue>

int main() {

    LidarDriver Driver;

    simple_socket::TCPServer Server(9998);
    std::atomic<bool> stopper(false);
    auto connection = Server.accept();
    std::cout << "Client connected" << std::endl;

    Communication com;
    std::vector<std::pair<double,double>> coordinates;
    std::mutex coordinates_mutex;

    Driver.startMotorHalf();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::queue<std::vector<std::pair<double,double>>> coordinates_queue;

    std::thread scanThread(
        &LidarDriver::startScan1,
        &Driver,
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

    std::thread controlThread([&stopper]() {
        std::cout << "Press Enter to stop scanning..." << std::endl;
        std::cin.get(); // Waits for the user to press Enter
        stopper = true;
    });

    scanThread.detach();
    senderThread.detach();
    controlThread.join(); // Join the control thread to wait for user input

    // After the control thread ends, stop the motor
    Driver.stopScan();
    Driver.stopMotor();
    Driver.close();
}
