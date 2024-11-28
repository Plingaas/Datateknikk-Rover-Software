//
// Created by peter on 29.10.24.
//

#ifndef ROVER_HPP
#define ROVER_HPP
#include <vector>
#include "RoverSerial.hpp"
#include "RoverSensorData.hpp"
#include "RoverStateStreamer.hpp"
#include <thread>
#include <queue>

namespace Rover {

    class Rover {
    private:
        void driveTank(const float& left_velocity, const float& right_velocity);
        void updateSensorData(std::shared_ptr<SerialMessage> msg);

        std::queue<std::vector<float>> stateQueue;
        // Sensors
        IMUData imu;
        AccelerometerData accelerometer;
        GyroscopeData gyroscope;
        LocatorData locator;
        VelocityData velocity;

        float state[13];

    public:
        // Serial communication
        std::unique_ptr<RoverSerial> device;
        std::unique_ptr<SerialParser> parser;
        std::unique_ptr<StateStreamer> stateStreamer;

        bool driving = false;

        static std::shared_ptr<Rover> create();

        Rover();
        void initialize();

        void init();
        void sleep();
        void wake();
        void stop();
        void resetYaw();
        void resetPosition();
        void driveRawMotors(const uint8_t& leftPWM, const uint8_t& rightPWM, const bool& leftDir = true, const bool& rightDir = true);
        void driveToPosition(const std::vector<float>& target, const float& linear_velocity, const float& final_heading, const uint8_t& flags = 0x00);
        void driveWithYaw(const float& linear_velocity, const float& heading);
        void startSensorStream(uint16_t period);
        void stopSensorStream();
        void clearSensorStream();
        void resetLocatorXY();
        void closeConnection();
        void update();
        const float* getState();

        [[nodiscard]] float getHeading() const {return this->imu.yaw;}


    };
}
#endif //ROVER_HPP
