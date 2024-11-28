//
// Created by peter on 31.10.24.
//

#ifndef ROVERSERIALSENSORSTREAM_HPP
#define ROVERSERIALSENSORSTREAM_HPP
#include <iostream>
#include <variant>
#include <vector>
#include <cmath>

using VariantFloatInt = std::variant<float, int>;
namespace Rover {
    enum SensorType {
      IMU = 0x01,
    };

    enum ValueTypes {
        INT = 0x00,
        FLOAT = 0x01,
    };

    struct SensorAttribute {
      std::string name;
      VariantFloatInt minValue;
      VariantFloatInt maxValue;
      uint8_t valueType;

      SensorAttribute(const std::string& name, const VariantFloatInt& minValue, const VariantFloatInt& maxValue, const uint8_t& valueType) :
                     name(name), minValue(minValue), maxValue(maxValue), valueType(valueType) {};

    };
}
#endif //ROVERSERIALSENSORSTREAM_HPP
