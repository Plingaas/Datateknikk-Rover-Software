//
// Created by peter on 29.10.24.
//

#include "Rover.hpp"

namespace Rover {

    Rover::Rover(uint16_t TCPPort) {
        device = std::make_unique<RoverSerial>();
        parser = std::make_unique<SerialParser>();
        streamer = std::make_unique<TCPStreamer>(TCPPort, true);
        streamer->setName("Rover");

        // Serial handler
        device->setOnNewDataHandler([this](std::vector<uint8_t>& data) {
            this->onSerialDataReceived(data);
        });


        // Server handlers
        streamer->setDisconnectHandler([this]() {
           this->onClientDisconnect();
        });

        streamer->setConnectHandler([this]() {
            this->onClientConnect();
        });

        streamer->setReconnectHandler([this] {
            this->onClientReconnect();
        });
    }

    void Rover::onClientConnect() {
        std::cout << "Rover client connected." << std::endl;
        startSerialSensorStream(33);
    }

    void Rover::onClientDisconnect() {
        std::cout << "Rover client disconnected." << std::endl;
        stopSensorStream();
        stopDriving();
        token1Received = false;
        token2Received = false;
    }

    void Rover::onClientReconnect() {
        std::cout << "Rover client reconnected." << std::endl;
    }



    void Rover::onSerialDataReceived(std::vector<uint8_t>& data) {
        parser->feed(data);
        auto msg = parser->parse();
        processSerialData(msg);
    }

    void Rover::connectSerial() const {
        if (!device->isConnected()) {
            device->open();
        }
    }

    void Rover::setServerTimeout(long milliseconds) const {
        streamer->setTimeout(milliseconds);
    }

    void Rover::startTCPServer() const {
        streamer->startStreaming();
    }

    void Rover::init() {
        wake();
        stopSensorStream();
        clearSensorStream();
        resetYaw();
        resetPosition();
    }
    void Rover::wake() {
        device->sendCommand("WAKE");
    }

    void Rover::stopDriving() {
        device->sendCommand("DRIVE_STOP");
        driving = false;
    }

    void Rover::resetYaw() {
        device->sendCommand("RESET_YAW");
    }

    void Rover::resetPosition() {
        device->sendCommand("RESET_LOCATOR_XY");
    }

    void Rover::driveRawMotors(const uint8_t& leftPWM, const uint8_t& rightPWM, const bool& leftDir, const bool& rightDir) {
        std::vector<VariantType> inputs{};
        leftDir ? inputs.emplace_back(static_cast<uint8_t>(0x01)) : inputs.emplace_back(static_cast<uint8_t>(0x02));
        inputs.emplace_back(leftPWM);
        rightDir ? inputs.emplace_back(static_cast<uint8_t>(0x01)) : inputs.emplace_back(static_cast<uint8_t>(0x02));
        inputs.emplace_back(rightPWM);

        device->sendCommand("DRIVE_RAW_MOTORS", inputs);
        driving = true;
    }

    void Rover::driveToPosition(const std::vector<float>& target, const float& linear_velocity, const float& final_heading, const uint8_t& flags) {

        std::vector<VariantType> inputs{};
        inputs.emplace_back(final_heading);
        inputs.insert(inputs.end(), target.begin(), target.end());
        inputs.emplace_back(linear_velocity);
        inputs.emplace_back(flags);

        device->sendCommand("DRIVE_TO_XY", inputs);
        driving = true;
    }
    void Rover::driveWithYaw(const float& linear_velocity, const float& heading) {
        std::vector<VariantType> inputs{};
        inputs.emplace_back(heading);
        inputs.emplace_back(linear_velocity);

        device->sendCommand("DRIVE_WITH_YAW", inputs);
        driving = true;
    }

    /*
     Commands rover to start streaming sensor data with period of ms specified.
     */
    void Rover::startSerialSensorStream(uint16_t period) {

        // Uglyyy, need to fix this but it works for now
        std::vector<VariantType> configInputs;

        // Add token 1
        configInputs.emplace_back(static_cast<uint8_t>(0x01));

        // IMU
        configInputs.emplace_back(static_cast<uint8_t>(0x00));
        configInputs.emplace_back(static_cast<uint8_t>(0x01));
        configInputs.emplace_back(static_cast<uint8_t>(0x02));

        // Accelerometer
        configInputs.emplace_back(static_cast<uint8_t>(0x00));
        configInputs.emplace_back(static_cast<uint8_t>(0x02));
        configInputs.emplace_back(static_cast<uint8_t>(0x02));

        // Gyroscope
        configInputs.emplace_back(static_cast<uint8_t>(0x00));
        configInputs.emplace_back(static_cast<uint8_t>(0x04));
        configInputs.emplace_back(static_cast<uint8_t>(0x02));

        // Send configuration for Token 1
        device->sendCommand("CONFIGURE_SENSOR_STREAM", configInputs);

        // Reset list and add token 2
        configInputs.clear();
        configInputs.emplace_back(static_cast<uint8_t>(0x02));

        // Locator
        configInputs.emplace_back(static_cast<uint8_t>(0x00));
        configInputs.emplace_back(static_cast<uint8_t>(0x06));
        configInputs.emplace_back(static_cast<uint8_t>(0x02));

        // Locator
        configInputs.emplace_back(static_cast<uint8_t>(0x00));
        configInputs.emplace_back(static_cast<uint8_t>(0x07));
        configInputs.emplace_back(static_cast<uint8_t>(0x02));
        device->sendCommand("CONFIGURE_SENSOR_STREAM", configInputs);

        std::vector<VariantType> inputs{};
        inputs.emplace_back(period);
        device->sendCommand("START_SENSOR_STREAM_1", inputs);
        device->sendCommand("START_SENSOR_STREAM_2", inputs);
    }

    void Rover::stopSensorStream() {
        device->sendCommand("STOP_SENSOR_STREAM_1"); // Stops Nordic processor
        device->sendCommand("STOP_SENSOR_STREAM_2"); // Stops ST processor
    }

    void Rover::resetLocatorXY() {
        device->sendCommand(("RESET_LOCATOR_XY"));
    }

    void Rover::sleep() {
        device->sendCommand("SLEEP");
        device->close();
    }

    void Rover::clearSensorStream() {
        device->sendCommand("CLEAR_SENSOR_STREAM_1");
        device->sendCommand("CLEAR_SENSOR_STREAM_2");
    }
    void Rover::closeConnection() {
        device->close();
    }

    const float* Rover::getState() {
        state[0] = imu.pitch;
        state[1] = imu.roll;
        state[2] = imu.yaw;
        state[3] = accelerometer.ax;
        state[4] = accelerometer.ay;
        state[5] = accelerometer.az;
        state[6] = gyroscope.gx;
        state[7] = gyroscope.gy;
        state[8] = gyroscope.gz;
        state[9] = locator.x;
        state[10] = locator.y;
        state[11] = velocity.x;
        state[12] = velocity.y;

        return state;
    }

    void Rover::processSerialData(std::shared_ptr<SerialMessage> msg) {
        if (msg == nullptr) return;
        //std::cout << "New response: DID: " << static_cast<int>(msg->header->did) << "  CID: " << static_cast<int>(msg->header->cid) << std::endl;

        // Notification for XY position move success or not
        if (msg->header->did == 22 && msg->header->cid == 58) {
            std::cout << "REACHED XY LOCATION (SUCCESS: " << static_cast<int>(msg->body[0]) << ")" << std::endl;
            driving = false;
        }

        // Sensor data
        if (msg->header->did == 24 && msg->header->cid == 61) {
            updateSensorData(msg);
            if (token1Received && token2Received) {
                auto packet = getStateAsPacket();
                streamer->addPacket(packet);
            }
        }
    }

    void Rover::updateSensorData(std::shared_ptr<SerialMessage> msg) {
        uint8_t token = msg->body.front();
        msg->body.erase(msg->body.begin());
        if (token == 0x01) {
            auto sensorData = parser->parseSensorData1(msg->body);

            imu.setData(sensorData[0]);
            accelerometer.setData(sensorData[1]);
            gyroscope.setData(sensorData[2]);
            token1Received = true;
        }
        if (token == 0x02) {
            auto sensorData = parser->parseSensorData2(msg->body);
            locator.setData(sensorData[0]);
            velocity.setData(sensorData[1]);
            token2Received = true;
        }
    }

    std::vector<uint8_t> Rover::getStateAsPacket() {
        auto data = getState();

        std::vector<uint8_t> packet{};
        for (int i = 12; i >= 0; i--) {
            uint8_t temp[sizeof(float)];
            std::memcpy(temp, &data[i], sizeof(float));
            packet.insert(packet.begin(), temp, temp + sizeof(float));
        }
        return packet;
    }
}