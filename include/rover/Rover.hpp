//
// Created by peter on 29.10.24.
//

#ifndef ROVER_HPP
#define ROVER_HPP
#include <vector>
#include "RoverSerial.hpp"
#include "RoverSensorData.hpp"
#include "RoverStateStreamer.hpp"
#include "TCPStreamer.hpp"
#include <thread>
#include <queue>

namespace Rover {

    class Rover {

    public:
        using SensorDataAvailableHandler = std::function<void()>;

        // Serial communication
        std::unique_ptr<RoverSerial> device;
        std::unique_ptr<SerialParser> parser;
        std::unique_ptr<TCPStreamer> streamer;

        //std::unique_ptr<StateStreamer> stateStreamer;

        bool driving = false;

        explicit Rover(uint16_t TCPPort);

        void connectSerial() const;
        void startTCPServer() const;
        void setServerTimeout(long milliseconds) const;

        // Server handlers
        void onClientDisconnect();
        void onClientConnect();
        void onClientReconnect();

        void init();
        void sleep();
        void wake();
        void stopDriving();
        void resetYaw();
        void resetPosition();
        void driveRawMotors(const uint8_t& leftPWM, const uint8_t& rightPWM, const bool& leftDir = true, const bool& rightDir = true);
        void driveToPosition(const std::vector<float>& target, const float& linear_velocity, const float& final_heading, const uint8_t& flags = 0x00);
        void driveWithYaw(const float& linear_velocity, const float& heading);
        void startSerialSensorStream(uint16_t period);
        void stopSensorStream();
        void clearSensorStream();
        void resetLocatorXY();
        void closeConnection();
        void processSerialData(std::shared_ptr<SerialMessage> msg);
        const float* getState();
        void driveTank(const float& left_velocity, const float& right_velocity);
        [[nodiscard]] float getHeading() const {return this->imu.yaw;}

    private:
        std::vector<uint8_t> getStateAsPacket();
        void onSerialDataReceived(std::vector<uint8_t>& data);

        void updateSensorData(std::shared_ptr<SerialMessage> msg);

        std::queue<std::vector<float>> stateQueue;
        // Sensors
        IMUData imu;
        AccelerometerData accelerometer;
        GyroscopeData gyroscope;
        LocatorData locator;
        VelocityData velocity;

        bool token1Received = false;
        bool token2Received = false;
        float state[13];

    };
}
#endif //ROVER_HPP
