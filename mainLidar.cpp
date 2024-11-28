#include "lidarDriver.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

int main() {

    // Open the serial port
    LidarDriver driver;
    serial::Serial serial("/dev/ttyUSB0", 256000);

    serial.setDTR(false); //Must be set to false for the communication to work.

    if (!serial.isOpen()) {
        std::cerr << "Failed to open serial port.\n";
        return 1;
    }

    // Send stop motor command
    driver.stopScan(serial);
    driver.stopMotor(serial);

    // Wait for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));
    serial.flush();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    // Send start motor command
    driver.startMotorFull(serial);

    // Wait for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::atomic<bool> stopper(false);

    std::thread scanThread(
        &LidarDriver::startScan2,
        &driver,
        std::ref(serial),
        std::ref(stopper));

    scanThread.join();

    std::this_thread::sleep_for(std::chrono::seconds(5));
    // Send stop motor command
    driver.stopScan(serial);
    driver.stopMotor(serial);

    //driver.reset(serial);


    // Close the serial port
    serial.close();

    return 0;

}

