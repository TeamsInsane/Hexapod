#pragma once

#include "HexapodConfig.h"

namespace Hexapod {
    // 4.17: zero velocity and acceleration at both ends
    inline float quinticSmoothStep(float u) {
        return u * u * u * (u * (u * 6.0F - 15.0F) + 10.0F);
    }

    inline bool servoInsideIkTestMargin(const ServoAnglesDeg &servo) {
        return servo.valid && servo.coxa >= kIkTestSafeServoMinDeg && servo.coxa <= kIkTestSafeServoMaxDeg && servo.
               femur >= kIkTestSafeServoMinDeg && servo.femur <= kIkTestSafeServoMaxDeg && servo.tibia >=
               kIkTestSafeServoMinDeg &&
               servo.tibia <= kIkTestSafeServoMaxDeg;
    }
} // namespace Hexapod
