#pragma once

#include "HexapodConfig.h"

namespace Hexapod {
    const char *aimDirectionName(float errorDegrees);
    const char *aimServoActionName(AimServoAction action);
    bool aimTrackingDataIsFresh(uint32_t nowMs);
    void applyAimServoAction(AimServoAction action);
    void updateAimServo();
    bool shootServoIsActive();
    void startShootServoPhase(ShootPhase phase);
    void stopShootServo();
    void updateShootServo();
    const char *tiltActionName(TiltAction action);
    bool tiltServoIsActive();
    void startTiltServo(TiltAction action, uint32_t durationMs);
    void stopTiltServo();
    void updateTiltServo();
    const char *chargeActionName(ChargeAction action);
    bool chargeServoIsActive();
    uint32_t chargeDurationForPowerLevel(uint8_t powerLevel);
    void startChargeServo(ChargeAction action, uint32_t durationMs = kChargeRunDurationMs);
    void stopChargeServo();
    void markChargeMechanismHome();
    void updateChargeServo();
    void cancelManualAim();
    void refreshPca9685Boards(bool force = false);
} // namespace Hexapod
