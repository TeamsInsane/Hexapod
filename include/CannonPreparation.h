#pragma once

#include <Arduino.h>

namespace Hexapod {
    bool cannonPreparationActive();
    bool cannonPreparationReady();
    bool cannonPreparationAwaitingConfirmation();
    bool cannonPreparationAwaitingTiltHomeConfirmation();
    bool cannonPreparationAwaitingTiltUpConfirmation();
    bool cannonPreparationAwaitingUnwindConfirmation();
    bool cannonPreparationCanAddChargeStep();
    bool cannonPreparationCanStart();
    const char *cannonPreparationStateName();
    const char *cannonPreparationReason();
    float cannonPreparationDistanceCm();
    uint16_t cannonPreparationTiltUpMs();
    uint8_t cannonPreparationPowerLevel();
    uint32_t cannonPreparationBaseChargeDurationMs();
    uint32_t cannonPreparationPlannedChargeDurationMs();
    int32_t cannonPreparationChargeCorrectionMs();
    bool startCannonPreparation(uint32_t nowMs);
    bool startAutomaticCannonPreparation(uint32_t nowMs);
    bool confirmCannonPreparationPlan();
    bool confirmCannonPreparationTiltFullyDown();
    bool confirmCannonPreparationTiltUp();
    bool addCannonPreparationChargeStep();
    bool confirmCannonPreparationChargedAndUnwind();
    void updateCannonPreparation(uint32_t nowMs);
    void cancelCannonPreparation(const char *reason = "cancelled by user");
    void cannonPreparationShotStarted();
} // namespace Hexapod
