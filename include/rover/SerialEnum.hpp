//
// Created by peter on 29.10.24.
//

#ifndef SERIALENUM_HPP
#define SERIALENUM_HPP
#include <bits/stdc++.h>
#include "RoverSerialHeader.hpp"

using namespace std;
namespace Rover {
    inline map<string, std::shared_ptr<SerialHeader>> serialCommands = {
        {"WAKE", std::make_shared<SerialHeader>(0x13, 0x0d, 0x01, 0x18)},
        {"SLEEP", std::make_shared<SerialHeader>(0x13, 0x01, 0x01, 0x18)},
        {"GET_BATTERY_PERCENTAGE", std::make_shared<SerialHeader>(0x13, 0x10, 0x01, 0x18)},
        {"RESET_YAW", std::make_shared<SerialHeader>(0x16, 0x06, 0x02, 0x18)},
        {"DRIVE_RAW_MOTORS", std::make_shared<SerialHeader>(0x16, 0x01, 0x02, 0x18)},
        {"DRIVE_WITH_HEADING", std::make_shared<SerialHeader>(0x16, 0x07, 0x02, 0x18)},
        {"DRIVE_TANK", std::make_shared<SerialHeader>(0x16, 0x32, 0x02, 0x18)},
        {"DRIVE_WITH_YAW", std::make_shared<SerialHeader>(0x16, 0x36, 0x02, 0x18)},
        {"DRIVE_TO_XY", std::make_shared<SerialHeader>(0x16, 0x38, 0x02, 0x18)},
        {"DRIVE_STOP", std::make_shared<SerialHeader>(0x16, 0x42, 0x02, 0x18)},
        {"SET_DRIVE_TARGET_SLEW_PARAMETERS", std::make_shared<SerialHeader>(0x16, 0x3c, 0x02, 0x18)},
        {"GET_DRIVE_TARGET_SLEW_PARAMETERS", std::make_shared<SerialHeader>(0x16, 0x3d, 0x02, 0x18)},
        {"RESET_LOCATOR_XY", std::make_shared<SerialHeader>(0x18, 0x13, 0x02, 0x18)},
        {"CONFIGURE_SENSOR_STREAM", std::make_shared<SerialHeader>(0x18, 0x39, 0x02, 0x18)},
        {"START_SENSOR_STREAM_1", std::make_shared<SerialHeader>(0x18, 0x3a, 0x01, 0x18)},
        {"START_SENSOR_STREAM_2", std::make_shared<SerialHeader>(0x18, 0x3a, 0x02, 0x18)},
        {"STOP_SENSOR_STREAM_1", std::make_shared<SerialHeader>(0x18, 0x3b, 0x01, 0x18)},
        {"STOP_SENSOR_STREAM_2", std::make_shared<SerialHeader>(0x18, 0x3b, 0x02, 0x18)},
        {"CLEAR_SENSOR_STREAM_1", std::make_shared<SerialHeader>(0x18, 0x3c, 0x01, 0x18)},
        {"CLEAR_SENSOR_STREAM_2", std::make_shared<SerialHeader>(0x18, 0x3c, 0x02, 0x18)},
    };

    enum UintBounds {
        UINT_8_MAX = 255,
        UINT_16_MAX = 65535,
        UINT_32_MAX = 4294967295
    };
}
#endif //SERIALENUM_HPP
