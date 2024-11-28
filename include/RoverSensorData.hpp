//
// Created by peter on 01.11.24.
//

#ifndef ROVERSENSORDATA_HPP
#define ROVERSENSORDATA_HPP

namespace Rover {

    struct IMUData {
        float pitch, roll, yaw;
        std::vector<uint8_t> data{};

        IMUData() {
            pitch = roll = yaw = 0.0f;
        }

        void setData(const std::vector<float>& data) {
            pitch = data[0];
            roll = data[1];
            yaw = data[2];
        }

        void setData(const IMUData& _data) {
            pitch = _data.pitch;
            roll = _data.roll;
            yaw = _data.yaw;
            data = _data.data;
        }
    };

    struct AccelerometerData {
        float ax, ay, az;
        std::vector<uint8_t> data{};
        AccelerometerData() {
            ax = ay = az = 0.0f;
        }

        void setData(const std::vector<float>& data) {
            ax = data[0];
            ay = data[1];
            az = data[2];
        }

        void setData(const AccelerometerData& _data) {
            ax = _data.ax;
            ay = _data.ay;
            az = _data.az;
            data = _data.data;
        }
    };

    struct GyroscopeData {
        float gx, gy, gz;
        std::vector<uint8_t> data{};
        GyroscopeData() {
            gx = gy = gz = 0.0f;
        }

        void setData(const std::vector<float>& data) {
            gx = data[0];
            gy = data[1];
            gz = data[2];
        }

        void setData(const GyroscopeData& _data) {
            gx = _data.gx;
            gy = _data.gy;
            gz = _data.gz;
            data = _data.data;
        }
    };

    struct LocatorData {
        float x, y;
        std::vector<uint8_t> data{};
        LocatorData() {
            x = y = 0.0f;
        }

        void setData(const std::vector<float>& data) {
            x = data[0];
            y = data[1];
        }

        void setData(const LocatorData& _data) {
            x = _data.x;
            y = _data.y;
            data = _data.data;
        }
    };

    struct VelocityData {
        float x, y;
        std::vector<uint8_t> data{};
        VelocityData() {
            x = y = 0.0f;
        }

        void setData(const std::vector<float>& data) {
            x = data[0];
            y = data[1];
        }

        void setData(const VelocityData& _data) {
            x = _data.x;
            y = _data.y;
            data = _data.data;
        }
    };
}
#endif //ROVERSENSORDATA_HPP
