#include "lidarDriver.hpp"
#include <iostream>
#include <thread>
#include <atomic>

int main() {

    LidarDriver driver;

    std::atomic<bool> stopper(false);

    serial::Serial serial("/dev/ttyUSB0",256000);

    serial.flush();
    serial.setDTR(false);

    if (!serial.isOpen()) {
        std::cerr << "Failed to open COM3\n";
        return 1;
    }

    std::thread controlThread([&stopper]() {
    std::cout << "Press Enter to stop scanning..." << std::endl;
    std::cin.get(); // Waits for the user to press Enter
    stopper = true;
     });

    driver.stopScan(serial);
    driver.stopMotor(serial);

    std::this_thread::sleep_for(std::chrono::seconds(3));
    serial.flush();
    driver.startMotorFull(serial);
    //Give the motor a few seconds to reach the desired speed
    std::this_thread::sleep_for(std::chrono::seconds(3));

    try {

        driver.startScan2(serial,stopper);

    } catch (const std::runtime_error& e) {
        std::cerr << "Caught an exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "An error occurred during the scan process." << std::endl;
    }


    driver.stopScan(serial);
    driver.stopMotor(serial);


    if (controlThread.joinable()) {
        controlThread.join();
    }

    serial.close();

    return 0;


}