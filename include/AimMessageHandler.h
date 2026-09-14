#pragma once

#include "HexapodConfig.h"

namespace Hexapod {
    bool readFloatArgument(const char *name, float &value);
    const ShotCalibrationPoint &findClosestShotCalibration(float distanceCm);
    bool readSequenceArgument(uint32_t &value);
    bool readBooleanArgument(const char *name, bool &value);
    void handleAimMessage();
    void handleAimLost();
} // namespace Hexapod
