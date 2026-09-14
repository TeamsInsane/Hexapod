#include <Arduino.h>
#include <Wire.h>

#include "HexapodState.h"
#include "CannonController.h"

namespace Hexapod {
namespace {

AimServoAction activeMicroPulseAction = AimServoAction::Stop;
uint32_t microPulseStopAtMs = 0U;
uint32_t nextMicroPulseAllowedAtMs = 0U;
uint32_t lastMicroPulseSequence = 0U;
AimServoAction pendingMicroPulseAction = AimServoAction::Stop;
uint8_t matchingMicroPulseFrames = 0U;

bool deadlineNotReached(uint32_t nowMs, uint32_t deadlineMs) {
    return static_cast<int32_t>(nowMs - deadlineMs) < 0;
}

void resetMicroPulseTracking() {
    activeMicroPulseAction = AimServoAction::Stop;
    microPulseStopAtMs = 0U;
    nextMicroPulseAllowedAtMs = 0U;
    lastMicroPulseSequence = aimState.latestSequence;
    pendingMicroPulseAction = AimServoAction::Stop;
    matchingMicroPulseFrames = 0U;
}

uint32_t microPulseDurationMs(float errorMagnitude) {
    if (errorMagnitude <= kAimMicroPulseFineRangeDegrees) {
        return kAimMicroPulseFineDurationMs;
    }

    if (errorMagnitude <= kAimMicroPulseMediumRangeDegrees) {
        return kAimMicroPulseMediumDurationMs;
    }

    return kAimMicroPulseCoarseDurationMs;
}

} // namespace

const char *aimDirectionName(float errorDegrees) {
    if (fabsf(errorDegrees) <= kAlignedToleranceDegrees) {
        return "aligned";
    }

    return errorDegrees > 0.0F ? "right" : "left";
}

const char *aimServoActionName(AimServoAction action) {
    switch (action) {
        case AimServoAction::Off:
            return "off";
        case AimServoAction::Stop:
            return "stop";
        case AimServoAction::Left:
            return "left";
        case AimServoAction::Right:
            return "right";
    }

    return "off";
}

bool aimTrackingDataIsFresh(uint32_t nowMs) {
    return aimState.hasMessage && aimState.tagsVisible && nowMs - aimState.lastMessageAtMs <= kAimControlFreshTimeoutMs;
}

bool i2cAddressResponds(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

int aimServoCommandForAction(AimServoAction action) {
    if (action == AimServoAction::Off) {
        return -1;
    }

    if (action == AimServoAction::Stop) {
        return kAimServoStopCommand;
    }

    if (cannonState.manualAimActive) {
        return action == AimServoAction::Left ? kAimServoStopCommand + kManualAimSpeedOffset : kAimServoStopCommand - kManualAimSpeedOffset;
    }

    const float errorMagnitude = fabsf(aimState.latestErrorDegrees);
    int speedOffset = kAimServoMaximumSpeedOffset;
    if (errorMagnitude <= kAimMicroPulseRangeDegrees) {
        speedOffset = kAimServoMinimumSpeedOffset;
    } else if (errorMagnitude < kAimServoSlowdownRangeDegrees) {
        speedOffset = static_cast<int>(lroundf( kAimServoMinimumSpeedOffset + (errorMagnitude - kAlignedToleranceDegrees) / (kAimServoSlowdownRangeDegrees - kAlignedToleranceDegrees) *
                                                (kAimServoMaximumSpeedOffset - kAimServoMinimumSpeedOffset) ));
        speedOffset = constrain( speedOffset, kAimServoMinimumSpeedOffset, kAimServoMaximumSpeedOffset );
    }

    return action == AimServoAction::Left ? kAimServoStopCommand + speedOffset : kAimServoStopCommand - speedOffset;
}

void applyAimServoAction(AimServoAction action) {
    const int command = aimServoCommandForAction(action);
    if (!pcaState.ready || (action == cannonState.appliedAimAction && command == cannonState.appliedAimCommand)) {
        return;
    }

    if (action == AimServoAction::Off) {
        aimPwm.setPWM(kAimServoChannel, 0, 4096);
        cannonState.appliedAimCommand = -1;
    } else {
        aimPwm.setPWM(kAimServoChannel, 0, RobotConfig::servoAngleToPulse(command));
        cannonState.appliedAimCommand = command;
    }

    cannonState.appliedAimAction = action;
    if (action == AimServoAction::Off) {
        Serial.printf("Aim CH%u off\n", kAimServoChannel);
    } else {
        Serial.printf( "Aim CH%u %s %d\n", kAimServoChannel, aimServoActionName(action), cannonState.appliedAimCommand );
    }
}

AimServoAction desiredAimServoAction(uint32_t nowMs) {
    //Manual
    if (cannonState.manualAimActive) {
        resetMicroPulseTracking();
        if (deadlineNotReached(nowMs, cannonState.manualAimStopAtMs)) {
            return cannonState.manualAimAction;
        }

        cannonState.manualAimActive = false;
        cannonState.manualAimAction = AimServoAction::Stop;
        cannonState.manualAimStopAtMs = 0;
    }

    //Tracking disabled
    if (!cannonState.aimTrackingEnabled) {
        resetMicroPulseTracking();
        return AimServoAction::Off;
    }

    //Error or aligned
    if (!aimTrackingDataIsFresh(nowMs)) {
        resetMicroPulseTracking();
        return AimServoAction::Off;
    }

    if (fabsf(aimState.latestErrorDegrees) <= kAlignedToleranceDegrees) {
        resetMicroPulseTracking();
        return AimServoAction::Off;
    }

    //Aligning
    const AimServoAction requestedAction = aimState.latestErrorDegrees > 0.0F ? AimServoAction::Right : AimServoAction::Left;
    const float errorMagnitude = fabsf(aimState.latestErrorDegrees);
    if (errorMagnitude > kAimMicroPulseRangeDegrees) {
        resetMicroPulseTracking();
        return requestedAction;
    }

    // Close to the target
    if (activeMicroPulseAction != AimServoAction::Stop) {
        if (requestedAction != activeMicroPulseAction) {
            resetMicroPulseTracking();
            nextMicroPulseAllowedAtMs = nowMs + kAimMicroPulsePauseMs;
            return AimServoAction::Off;
        }

        if (deadlineNotReached(nowMs, microPulseStopAtMs)) {
            return activeMicroPulseAction;
        }

        activeMicroPulseAction = AimServoAction::Stop;
        microPulseStopAtMs = 0U;
        nextMicroPulseAllowedAtMs = nowMs + kAimMicroPulsePauseMs;
        pendingMicroPulseAction = AimServoAction::Stop;
        matchingMicroPulseFrames = 0U;
        return AimServoAction::Off;
    }

    if (deadlineNotReached(nowMs, nextMicroPulseAllowedAtMs)) {
        return AimServoAction::Off;
    }

    if (aimState.latestSequence != lastMicroPulseSequence) {
        lastMicroPulseSequence = aimState.latestSequence;

        if (requestedAction == pendingMicroPulseAction) {
            if (matchingMicroPulseFrames < kAimMicroPulseRequiredFrames) {
                ++matchingMicroPulseFrames;
            }
        } else {
            pendingMicroPulseAction = requestedAction;
            matchingMicroPulseFrames = 1U;
        }
    }

    if (matchingMicroPulseFrames < kAimMicroPulseRequiredFrames) {
        return AimServoAction::Off;
    }

    activeMicroPulseAction = requestedAction;
    microPulseStopAtMs = nowMs + microPulseDurationMs(errorMagnitude);
    pendingMicroPulseAction = AimServoAction::Stop;
    matchingMicroPulseFrames = 0U;
    return activeMicroPulseAction;
}

void updateAimServo() {
    applyAimServoAction(desiredAimServoAction(millis()));
}

const char *shootServoPhaseName(ShootPhase phase) {
    switch (phase) {
        case ShootPhase::Idle:
            return "idle";
        case ShootPhase::Forward:
            return "forward";
        case ShootPhase::Reset:
            return "reset";
    }
    return "idle";
}

bool shootServoIsActive() {
    return cannonState.shootPhase != ShootPhase::Idle;
}

void startShootServoPhase(ShootPhase phase) {
    const int command = phase == ShootPhase::Forward ? kShootForwardCommand : kShootResetCommand;
    aimPwm.setPWM(kShootServoChannel, 0, RobotConfig::servoAngleToPulse(command));
    cannonState.shootPhase = phase;
    cannonState.shootStopAtMs = millis() + kShootPhaseDurationMs;
    Serial.printf( "Shoot CH%u %s %d %lu ms\n", kShootServoChannel, shootServoPhaseName(phase), command, static_cast<unsigned long>(kShootPhaseDurationMs) );
}

void stopShootServo() {
    if (pcaState.ready) {
        aimPwm.setPWM(kShootServoChannel, 0, 4096);
    }

    cannonState.shootPhase = ShootPhase::Idle;
    cannonState.shootReturnAfterForward = false;
    cannonState.shootStopAtMs = 0;
}

void updateShootServo() {
    if (!shootServoIsActive() || static_cast<int32_t>(millis() - cannonState.shootStopAtMs) < 0) {
        return;
    }

    if (cannonState.shootPhase == ShootPhase::Forward && cannonState.shootReturnAfterForward) {
        cannonState.shootReturnAfterForward = false;
        startShootServoPhase(ShootPhase::Reset);
        return;
    }

    stopShootServo();
}

const char *tiltActionName(TiltAction action) {
    switch (action) {
        case TiltAction::Idle:
            return "idle";
        case TiltAction::Up:
            return "up";
        case TiltAction::Down:
            return "down";
    }

    return "idle";
}

bool tiltServoIsActive() {
    return cannonState.tiltAction != TiltAction::Idle;
}

void startTiltServo(TiltAction action, uint32_t durationMs) {
    const int command = action == TiltAction::Up ? kTiltUpCommand : kTiltDownCommand;
    aimPwm.setPWM(kTiltServoChannel, 0, RobotConfig::servoAngleToPulse(command));
    cannonState.tiltAction = action;
    cannonState.tiltStopAtMs = millis() + durationMs;
}

void stopTiltServo() {
    if (pcaState.ready) {
        aimPwm.setPWM(kTiltServoChannel, 0, 4096);
    }

    cannonState.tiltAction = TiltAction::Idle;
    cannonState.tiltStopAtMs = 0;
}

void updateTiltServo() {
    if (tiltServoIsActive() && static_cast<int32_t>(millis() - cannonState.tiltStopAtMs) >= 0) {
        stopTiltServo();
    }
}

const char *chargeActionName(ChargeAction action) {
    switch (action) {
        case ChargeAction::Idle:
            return "idle";
        case ChargeAction::Charge:
            return "charge";
        case ChargeAction::Uncharge:
            return "uncharge";
    }

    return "idle";
}

bool chargeServoIsActive() {
    return cannonState.chargeAction != ChargeAction::Idle;
}

uint32_t chargeDurationForPowerLevel(uint8_t powerLevel) {
    switch (powerLevel) {
        case 1U:
            return kChargeP1DurationMs;
        case 2U:
            return kChargeP2DurationMs;
        case 3U:
            return kChargeP3DurationMs;
        default:
            return 0U;
    }
}

void startChargeServo(ChargeAction action, uint32_t durationMs) {
    if (action == ChargeAction::Idle || durationMs == 0U) {
        stopChargeServo();
        return;
    }

    durationMs = min(durationMs, action == ChargeAction::Charge ? kMaximumChargeDurationMs : kMaximumUnchargeDurationMs);
    const int command = action == ChargeAction::Charge ? kChargeCommand : kUnchargeCommand;
    aimPwm.setPWM(kChargeServoChannel, 0, RobotConfig::servoAngleToPulse(command));
    cannonState.chargeAction = action;
    cannonState.chargeStartedAtMs = millis();
    cannonState.chargePlannedDurationMs = durationMs;
    cannonState.chargeStopAtMs = cannonState.chargeStartedAtMs + durationMs;
}

void stopChargeServo() {
    const ChargeAction stoppedAction = cannonState.chargeAction;
    const uint32_t elapsedMs = chargeServoIsActive() ? min(static_cast<uint32_t>(millis() - cannonState.chargeStartedAtMs), cannonState.chargePlannedDurationMs) : 0U;
    if (pcaState.ready) {
        aimPwm.setPWM(kChargeServoChannel, 0, 4096);
    }

    if (stoppedAction == ChargeAction::Charge) {
        cannonState.estimatedWoundDurationMs = min( cannonState.estimatedWoundDurationMs + elapsedMs, kMaximumChargeDurationMs);
    } else if (stoppedAction == ChargeAction::Uncharge) {
        cannonState.estimatedWoundDurationMs = elapsedMs >= cannonState.estimatedWoundDurationMs ? 0U : cannonState.estimatedWoundDurationMs - elapsedMs;
    }

    cannonState.chargeAction = ChargeAction::Idle;
    cannonState.chargeStopAtMs = 0;
    cannonState.chargeStartedAtMs = 0;
    cannonState.chargePlannedDurationMs = 0;
}

void markChargeMechanismHome() {
    if (chargeServoIsActive()) {
        return;
    }

    cannonState.estimatedWoundDurationMs = 0U;
}

void updateChargeServo() {
    if (chargeServoIsActive() && static_cast<int32_t>(millis() - cannonState.chargeStopAtMs) >= 0) {
        stopChargeServo();
    }
}

void cancelManualAim() {
    cannonState.manualAimActive = false;
    cannonState.manualAimAction = AimServoAction::Stop;
    cannonState.manualAimStopAtMs = 0;
}

void refreshPca9685Boards(bool force) {
    const uint32_t nowMs = millis();
    if (!force && nowMs - pcaState.lastCheckAtMs < kPcaCheckIntervalMs) {
        return;
    }

    pcaState.lastCheckAtMs = nowMs;
    if (!i2cAddressResponds(RobotConfig::kPcaDefaultAddress)) {
        if (!pcaState.checked || pcaState.ready) {
            Serial.println("ERROR: non-soldered PCA9685 0x40 not found");
        }

        pcaState.ready = false;
        cannonState.aimTrackingEnabled = false;
        cannonState.manualAimActive = false;
        cannonState.manualAimAction = AimServoAction::Stop;
        cannonState.manualAimStopAtMs = 0;
        cannonState.shootPhase = ShootPhase::Idle;
        cannonState.shootReturnAfterForward = false;
        cannonState.shootStopAtMs = 0;
        cannonState.tiltAction = TiltAction::Idle;
        cannonState.tiltStopAtMs = 0;
        stopChargeServo();
        cannonState.appliedAimAction = AimServoAction::Off;
        cannonState.appliedAimCommand = -1;
    } else if (!pcaState.ready) {
        aimPwm.begin();
        aimPwm.setPWMFreq(kServoFrequencyHz);
        delay(10);
        pcaState.ready = true;

        for (uint8_t channel = 0; channel < 16U; ++channel) {
            aimPwm.setPWM(channel, 0, 4096);
        }
        cannonState.appliedAimAction = AimServoAction::Off;
        cannonState.appliedAimCommand = -1;
        Serial.printf( "Non-soldered PCA9685 0x40 ready");
    }

    if (!i2cAddressResponds(RobotConfig::kPcaSolderedAddress)) {
        if (!pcaState.checked || pcaState.solderedReady) {
            Serial.println("ERROR: soldered PCA9685 0x41 not found.");
        }

        pcaState.solderedReady = false;
    } else if (!pcaState.solderedReady) {
        solderedPwm.begin();
        solderedPwm.setPWMFreq(kServoFrequencyHz);
        delay(10);
        pcaState.solderedReady = true;

        for (uint8_t channel = 0; channel < 16U; ++channel) {
            solderedPwm.setPWM(channel, 0, 4096);
        }
        Serial.println("Soldered PCA9685 0x41 ready");
    }
    pcaState.checked = true;
}

} // namespace Hexapod
