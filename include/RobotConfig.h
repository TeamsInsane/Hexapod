#pragma once

#include <Arduino.h>
#include "RobotTypes.h"

namespace RobotConfig {
    constexpr uint8_t kLegCount = 6;
    constexpr uint8_t kJointCount = 3;
    constexpr uint8_t kToFSensorCount = 3;

    constexpr uint16_t kServoPulseMin = 150;
    constexpr uint16_t kServoPulseMax = 470;
    constexpr uint16_t kServoFrequencyHz = 50;
    constexpr unsigned long kSensorUpdateIntervalMs = 40;

    constexpr uint8_t kI2cSda = 8;
    constexpr uint8_t kI2cScl = 9;
    constexpr uint8_t kPcaDefaultAddress = 0x40;
    constexpr uint8_t kPcaSolderedAddress = 0x41;
    constexpr uint8_t kPca9548aAddress = 0x71;

    inline int servoAngleToPulse(int angle) {
        return map(constrain(angle, 0, 180), 0, 180, kServoPulseMin, kServoPulseMax);
    }

    // Physically VL53L0X mux mapping
    // CH1 front, CH7 left, CH0 right
    struct ToFSensorConfig {
        uint8_t muxChannel;
        const char *name;
    };

    constexpr ToFSensorConfig kToFSensors[kToFSensorCount] = {
        {1, "front"},
        {7, "left"},
        {0, "right"},
    };

    constexpr uint16_t kFrontHardStopMm = 250;
    constexpr uint16_t kFrontTurnTriggerMm = 450;
    constexpr uint8_t kObstacleConfirmSamples = 3;
    constexpr uint16_t kFrontClearMm = 650;
    constexpr uint8_t kClearConfirmSamples = 5;
    constexpr uint16_t kSideBlockedMm = 350;
    constexpr uint16_t kSideDirectionMarginMm = 100;
    constexpr unsigned long kSensorStaleTimeoutMs = 500;
    constexpr unsigned long kSensorReinitializeDelayMs = 5000;

    enum class PwmBoard : uint8_t {
        Soldered,
        Default,
    };

    struct ServoMapping {
        PwmBoard board;
        uint8_t channel;
    };

    struct LegServoConfig {
        const char *name;
        ServoMapping joints[kJointCount];
    };

    constexpr LegServoConfig kLegServos[kLegCount] = {
        {"LF", {{PwmBoard::Soldered, 13}, {PwmBoard::Soldered, 14}, {PwmBoard::Soldered, 15}}},
        {"LM", {{PwmBoard::Soldered, 12}, {PwmBoard::Soldered, 11}, {PwmBoard::Soldered, 10}}},
        {"LR", {{PwmBoard::Soldered, 1}, {PwmBoard::Soldered, 2}, {PwmBoard::Soldered, 0}}},
        {"RF", {{PwmBoard::Default, 2}, {PwmBoard::Default, 1}, {PwmBoard::Default, 0}}},
        {"RM", {{PwmBoard::Default, 12}, {PwmBoard::Default, 11}, {PwmBoard::Default, 10}}},
        {"RR", {{PwmBoard::Default, 13}, {PwmBoard::Default, 14}, {PwmBoard::Default, 15}}},
    };

    // model -> raw 
    struct JointCalibration {
        float neutralDeg;
        float direction;
        float offsetDeg;
        float minDeg;
        float maxDeg;
    };

    // servo = neutral + (kinematic - offset) / direction.
    constexpr JointCalibration kJointCalibration[kLegCount][kJointCount] = {
        {{90, 1, 0, 0, 180}, {90, 1, 0, 0, 180}, {90, -1, -90, 0, 180}},
        {{90, 1, 0, 0, 180}, {90, 1, 0, 0, 180}, {90, -1, -90, 0, 180}},
        {{90, 1, 0, 0, 180}, {90, 1, 0, 0, 180}, {90, -1, -90, 0, 180}},
        {{90, 1, 0, 0, 180}, {90, 1, 0, 0, 180}, {90, -1, -90, 0, 180}},
        {{90, 1, 0, 0, 180}, {90, 1, 0, 0, 180}, {90, -1, -90, 0, 180}},
        {{90, 1, 0, 0, 180}, {90, 1, 0, 0, 180}, {90, -1, -90, 0, 180}},
    };

    struct LegGeometry {
        Vec3 coxaBodyMm;
        Vec3 homeFootBodyMm;
        float coxaMountYawRad;
    };

    // Leg dimensions
    constexpr float kCoxaLengthMm = 38.0F;
    constexpr float kFemurLengthMm = 86.25F;
    constexpr float kTibiaLengthMm = 160.5F;

    //1. Kje je coxa/hip sklep na telesu.
    //2. Kje je referenčno stopalo v body-frame.
    //3. Pod katerim kotom je noga nameščena.
    constexpr LegGeometry kLegGeometry[kLegCount] = {
        {{108.445929F, 67.108377F, 0.0F}, {256.358509F, 158.639459F, -121.321428F}, 0.554142F},
        {{0.0F, 115.7222F, 0.0F}, {0.0F, 289.664922F, -121.321428F}, 1.570796F},
        {{-108.445929F, 67.108377F, 0.0F}, {-256.358509F, 158.639459F, -121.321428F}, 2.587451F},
        {{108.445929F, -67.108377F, 0.0F}, {256.358509F, -158.639459F, -121.321428F}, -0.554142F},
        {{0.0F, -115.7222F, 0.0F}, {0.0F, -289.664922F, -121.321428F}, -1.570796F},
        {{-108.445929F, -67.108377F, 0.0F}, {-256.358509F, -158.639459F, -121.321428F}, -2.587451F},
    };
} // namespace RobotConfig
