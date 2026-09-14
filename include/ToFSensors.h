#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "RobotTypes.h"

namespace Hexapod {
    bool beginToFSensors(TwoWire &wire);
    void updateToFSensors(uint32_t nowMs);
    bool tofSensorsStarted();
    const DistanceReading &tofSensorReading(uint8_t index);
    bool tofSensorInitialized(uint8_t index);
    const char *tofSensorFault();
} // namespace Hexapod
