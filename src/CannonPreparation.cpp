#include "CannonPreparation.h"

#include <stdarg.h>
#include <stdio.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "AimMessageHandler.h"
#include "CannonController.h"

namespace Hexapod {
namespace {

enum class PreparationState : uint8_t {
    Idle,
    WaitingForAim,
    Aiming,
    Locking,
    WaitingForDistance,
    AwaitingConfirmation,
    HomingTilt,
    AwaitingTiltHomeConfirmation,
    ApplyingTilt,
    AwaitingTiltUpConfirmation,
    Charging,
    AwaitingUnwindConfirmation,
    AdjustingCharge,
    UnwindingCharge,
    Ready,
    Fault,
};

PreparationState state = PreparationState::Idle;
char statusReason[192] = "automatic preparation not enabled";
uint32_t lastAimSequence = 0;
uint32_t lastAimLockSampleMs = 0;
uint8_t aimLockCount = 0;
float capturedDistanceCm = 0.0F;
uint16_t selectedTiltUpMs = 0;
uint8_t selectedPowerLevel = 0;
uint32_t baseChargeDurationMs = 0;
uint32_t plannedChargeDurationMs = 0;
int32_t pendingChargeCorrectionMs = 0;
bool calibratedChargeRunStarted = false;
bool fullyAutomaticSequence = false;

void setReason(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(statusReason, sizeof(statusReason), format, args);
    va_end(args);
}

bool cameraTargetDistanceAvailable(uint32_t nowMs, float &distanceCm) {
    if (!aimTrackingDataIsFresh(nowMs) || !aimState.hasTargetDistance || aimState.targetDistanceAtMs == 0U || nowMs - aimState.targetDistanceAtMs > kAimControlFreshTimeoutMs) {
        return false;
    }

    distanceCm = aimState.targetDistanceCm;
    return distanceCm >= kMinimumCalibratedRangeCm && distanceCm <= kMaximumCalibratedRangeCm;
}

void stopAutomaticOutputs() {
    cannonState.aimTrackingEnabled = false;
    cannonState.manualAimActive = false;
    cannonState.manualAimAction = AimServoAction::Stop;
    cannonState.manualAimStopAtMs = 0;
    applyAimServoAction(AimServoAction::Off);
    stopTiltServo();
    stopChargeServo();
}

void failPreparation(const char *message) {
    stopAutomaticOutputs();
    state = PreparationState::Fault;
    setReason("%s", message);
    Serial.printf("Cannon fault: %s\n", message);
}

void beginCharging() {
    calibratedChargeRunStarted = false;
    state = PreparationState::Charging;
    setReason("Charging P%u: %lu ms", static_cast<unsigned int>(selectedPowerLevel), static_cast<unsigned long>(plannedChargeDurationMs));
}

void beginTiltHoming() {
    startTiltServo(TiltAction::Down, kCannonTiltHomeDurationMs);
    state = PreparationState::HomingTilt;
    setReason("Tilt down: %lu ms", static_cast<unsigned long>(kCannonTiltHomeDurationMs));
}

void continueAfterTilt() {
    if (fullyAutomaticSequence) {
        beginCharging();
    } else {
        state = PreparationState::AwaitingTiltUpConfirmation;
        setReason("Confirm tilt before charging");
    }
}

void applySelectedTiltOrCharge() {
    if (selectedTiltUpMs > 0U) {
        startTiltServo(TiltAction::Up, selectedTiltUpMs);
        state = PreparationState::ApplyingTilt;
        setReason("Tilt up: %u ms", static_cast<unsigned int>(selectedTiltUpMs));
    } else {
        continueAfterTilt();
    }
}

void beginRecordedUnwind() {
    const uint32_t unwindDurationMs = min(cannonState.estimatedWoundDurationMs + kRecordedUnwindExtraDurationMs, kMaximumUnchargeDurationMs);
    startChargeServo(ChargeAction::Uncharge, unwindDurationMs);
    state = PreparationState::UnwindingCharge;
    setReason("Unwinding: %lu ms", static_cast<unsigned long>(unwindDurationMs));
}

void captureShotSetting(uint32_t nowMs) {
    float distanceCm = 0.0F;
    if (!cameraTargetDistanceAvailable(nowMs, distanceCm)) {
        state = PreparationState::WaitingForDistance;
        setReason("Waiting for distance (%.0f-%.0f cm)", static_cast<double>(kMinimumCalibratedRangeCm), static_cast<double>(kMaximumCalibratedRangeCm));
        return;
    }

    const ShotCalibrationPoint &solution = findClosestShotCalibration(distanceCm);
    capturedDistanceCm = distanceCm;
    selectedTiltUpMs = solution.tiltUpMs;
    selectedPowerLevel = solution.powerLevel;
    baseChargeDurationMs = chargeDurationForPowerLevel(selectedPowerLevel);
    plannedChargeDurationMs = baseChargeDurationMs;
    pendingChargeCorrectionMs = 0;
    calibratedChargeRunStarted = false;

    if (fullyAutomaticSequence) {
        beginTiltHoming();
    } else {
        state = PreparationState::AwaitingConfirmation;
        setReason("Plan: %.1f cm, tilt %u ms, P%u", static_cast<double>(capturedDistanceCm), static_cast<unsigned int>(selectedTiltUpMs), static_cast<unsigned int>(selectedPowerLevel));
    }
}

void updateAimAcquisition(uint32_t nowMs) {
    //Not fresh
    if (!aimTrackingDataIsFresh(nowMs)) {
        state = PreparationState::WaitingForAim;

        // If we lose connection/data, dont delete the old frames yet
        const bool retainPartialLock = aimLockCount > 0U && lastAimLockSampleMs != 0U && nowMs - lastAimLockSampleMs <= kAimLockMaximumSampleGapMs;
        if (!retainPartialLock) {
            aimLockCount = 0U;
            lastAimLockSampleMs = 0U;
        }

        setReason(aimState.tagsVisible ? "Waiting for aim data: %u/%u" : retainPartialLock ? "Tag lost; lock %u/%u" : "Tags not visible",
                  static_cast<unsigned int>(aimLockCount),
                  static_cast<unsigned int>(kCannonAimLockSamples));
        return;
    }

    if (fabsf(aimState.latestErrorDegrees) > kAlignedToleranceDegrees) {
        state = PreparationState::Aiming;
        aimLockCount = 0U;
        lastAimLockSampleMs = 0U;
        setReason("Aiming: %.2f deg", static_cast<double>(aimState.latestErrorDegrees));
        return;
    }

    if (aimState.latestSequence != lastAimSequence) {
        if (lastAimLockSampleMs != 0U && aimState.lastMessageAtMs - lastAimLockSampleMs > kAimLockMaximumSampleGapMs) {
            aimLockCount = 0U;
        }

        lastAimSequence = aimState.latestSequence;
        lastAimLockSampleMs = aimState.lastMessageAtMs;
        if (aimLockCount < kCannonAimLockSamples) {
            ++aimLockCount;
        }
    }

    state = PreparationState::Locking;
    setReason("Aim lock: %u/%u", static_cast<unsigned int>(aimLockCount), static_cast<unsigned int>(kCannonAimLockSamples));
    if (aimLockCount >= kCannonAimLockSamples) {
        captureShotSetting(nowMs);
    }
}

bool mechanismBusy() {
    return shootServoIsActive() || tiltServoIsActive() || chargeServoIsActive();
}

} // namespace

bool cannonPreparationActive() {
    return state != PreparationState::Idle;
}

bool cannonPreparationReady() {
    const uint32_t nowMs = millis();
    return state == PreparationState::Ready && aimTrackingDataIsFresh(nowMs) && fabsf(aimState.latestErrorDegrees) <= kAlignedToleranceDegrees;
}

bool cannonPreparationAwaitingConfirmation() {
    return state == PreparationState::AwaitingConfirmation;
}

bool cannonPreparationAwaitingTiltHomeConfirmation() {
    return state == PreparationState::AwaitingTiltHomeConfirmation;
}

bool cannonPreparationAwaitingTiltUpConfirmation() {
    return state == PreparationState::AwaitingTiltUpConfirmation;
}

bool cannonPreparationAwaitingUnwindConfirmation() {
    return state == PreparationState::AwaitingUnwindConfirmation;
}

bool cannonPreparationCanAddChargeStep() {
    return state == PreparationState::AwaitingUnwindConfirmation &&
           pcaState.ready &&
           !chargeServoIsActive() &&
           plannedChargeDurationMs <= kMaximumChargeDurationMs - kCannonPreparationChargeAdjustmentMs &&
           cannonState.estimatedWoundDurationMs <= kMaximumChargeDurationMs - kCannonPreparationChargeAdjustmentMs;
}

bool cannonPreparationCanStart() {
    return state == PreparationState::Idle && pcaState.ready && !legOperationIsActive() && !cannonState.manualAimActive && cannonState.estimatedWoundDurationMs == 0U && !mechanismBusy();
}

const char *cannonPreparationStateName() {
    switch (state) {
        case PreparationState::Idle: return "idle";
        case PreparationState::WaitingForAim: return "waiting for aim data";
        case PreparationState::Aiming: return "aiming";
        case PreparationState::Locking: return "checking alignment";
        case PreparationState::WaitingForDistance: return "waiting for distance";
        case PreparationState::AwaitingConfirmation: return "awaiting confirmation";
        case PreparationState::HomingTilt: return "moving tilt fully down";
        case PreparationState::AwaitingTiltHomeConfirmation: return "confirm tilt fully down";
        case PreparationState::ApplyingTilt: return "setting tilt";
        case PreparationState::AwaitingTiltUpConfirmation: return "confirm tilt";
        case PreparationState::Charging: return "charging";
        case PreparationState::AwaitingUnwindConfirmation: return "confirm charged then unwind";
        case PreparationState::AdjustingCharge: return "adjusting charge";
        case PreparationState::UnwindingCharge: return "unwinding charge line";
        case PreparationState::Ready: return "ready to shoot";
        case PreparationState::Fault: return "fault";
    }
    return "idle";
}

const char *cannonPreparationReason() {
    return statusReason;
}

float cannonPreparationDistanceCm() {
    return capturedDistanceCm;
}

uint16_t cannonPreparationTiltUpMs() {
    return selectedTiltUpMs;
}

uint8_t cannonPreparationPowerLevel() {
    return selectedPowerLevel;
}

uint32_t cannonPreparationBaseChargeDurationMs() {
    return baseChargeDurationMs;
}

uint32_t cannonPreparationPlannedChargeDurationMs() {
    return plannedChargeDurationMs;
}

int32_t cannonPreparationChargeCorrectionMs() {
    return static_cast<int32_t>(plannedChargeDurationMs) - static_cast<int32_t>(baseChargeDurationMs);
}

static bool startCannonPreparationMode(uint32_t nowMs, bool fullyAutomatic) {
    if (!cannonPreparationCanStart()) {
        return false;
    }

    cancelManualAim();
    stopShootServo();
    stopTiltServo();
    stopChargeServo();

    capturedDistanceCm = 0.0F;
    selectedTiltUpMs = 0U;
    selectedPowerLevel = 0U;
    baseChargeDurationMs = 0U;
    plannedChargeDurationMs = 0U;
    pendingChargeCorrectionMs = 0;
    calibratedChargeRunStarted = false;
    fullyAutomaticSequence = fullyAutomatic;
    aimLockCount = 0U;
    lastAimLockSampleMs = 0U;
    lastAimSequence = aimState.latestSequence;
    cannonState.aimTrackingEnabled = true;
    state = PreparationState::WaitingForAim;
    setReason(fullyAutomatic ? "Auto setup started" : "Aim capture started");
    updateAimServo();
    Serial.println(fullyAutomatic ? "Cannon auto start" : "Cannon capture start");
    return true;
}

bool startCannonPreparation(uint32_t nowMs) {
    return startCannonPreparationMode(nowMs, false);
}

bool startAutomaticCannonPreparation(uint32_t nowMs) {
    return startCannonPreparationMode(nowMs, true);
}

bool confirmCannonPreparationPlan() {
    if (state != PreparationState::AwaitingConfirmation || plannedChargeDurationMs == 0 || cannonState.estimatedWoundDurationMs != 0U || !aimTrackingDataIsFresh(millis()) ||
        fabsf(aimState.latestErrorDegrees) > kAlignedToleranceDegrees || mechanismBusy()) {
        return false;
    }

    beginTiltHoming();
    return true;
}

bool confirmCannonPreparationTiltFullyDown() {
    if (state != PreparationState::AwaitingTiltHomeConfirmation || tiltServoIsActive()) {
        return false;
    }

    applySelectedTiltOrCharge();
    return true;
}

bool confirmCannonPreparationTiltUp() {
    if (state != PreparationState::AwaitingTiltUpConfirmation || tiltServoIsActive() || chargeServoIsActive()) {
        return false;
    }

    beginCharging();
    return true;
}

bool addCannonPreparationChargeStep() {
    if (!cannonPreparationCanAddChargeStep()) {
        return false;
    }

    pendingChargeCorrectionMs = static_cast<int32_t>(kCannonPreparationChargeAdjustmentMs);
    startChargeServo(ChargeAction::Charge, kCannonPreparationChargeAdjustmentMs);
    state = PreparationState::AdjustingCharge;
    setReason("Adding %lu ms; total %lu ms",
              static_cast<unsigned long>(kCannonPreparationChargeAdjustmentMs),
              static_cast<unsigned long>(plannedChargeDurationMs + kCannonPreparationChargeAdjustmentMs));
    return true;
}

bool confirmCannonPreparationChargedAndUnwind() {
    if (state != PreparationState::AwaitingUnwindConfirmation || chargeServoIsActive() || cannonState.estimatedWoundDurationMs == 0U) {
        return false;
    }

    beginRecordedUnwind();
    return true;
}

void updateCannonPreparation(uint32_t nowMs) {
    if (state == PreparationState::Idle || state == PreparationState::Fault) {
        return;
    }

    if (!pcaState.ready) {
        failPreparation("PCA9685 disconnected");
        return;
    }

    if (legOperationIsActive()) {
        failPreparation("robot leg movement active, cannon preparation cancelled");
        return;
    }

    if (cannonState.manualAimActive) {
        failPreparation("manual servo command active, cannon preparation cancelled");
        return;
    }

    switch (state) {
        case PreparationState::WaitingForAim:
        case PreparationState::Aiming:
        case PreparationState::Locking:
        case PreparationState::WaitingForDistance:
            updateAimAcquisition(nowMs);
            break;

        case PreparationState::AwaitingConfirmation:
            if (!aimTrackingDataIsFresh(nowMs)) {
                setReason("Waiting for fresh tags");
            } else {
                setReason("Plan: %.1f cm, tilt %u ms, P%u", static_cast<double>(capturedDistanceCm), static_cast<unsigned int>(selectedTiltUpMs), static_cast<unsigned int>(selectedPowerLevel));
            }
            break;

        case PreparationState::HomingTilt:
            if (!tiltServoIsActive()) {
                if (fullyAutomaticSequence) {
                    applySelectedTiltOrCharge();
                } else {
                    state = PreparationState::AwaitingTiltHomeConfirmation;
                    setReason("Tilt down complete");
                }
            }

            break;
        case PreparationState::AwaitingTiltHomeConfirmation:
            if (!tiltServoIsActive()) {
                setReason("Confirm tilt down");
            }
            break;

        case PreparationState::ApplyingTilt:
            if (!tiltServoIsActive()) {
                continueAfterTilt();
            }
            break;

        case PreparationState::AwaitingTiltUpConfirmation:
            break;

        case PreparationState::Charging:
            if (!chargeServoIsActive()) {
                if (!calibratedChargeRunStarted) {
                    if (plannedChargeDurationMs == 0U || plannedChargeDurationMs > kMaximumChargeDurationMs) {
                        failPreparation("Outside of allowed duration range");
                        break;
                    }

                    calibratedChargeRunStarted = true;
                    startChargeServo(ChargeAction::Charge, plannedChargeDurationMs);
                    setReason("Charging P%u: %lu ms",
                              static_cast<unsigned int>(selectedPowerLevel),
                              static_cast<unsigned long>(plannedChargeDurationMs));
                } else if (fullyAutomaticSequence) {
                    beginRecordedUnwind();
                } else {
                    state = PreparationState::AwaitingUnwindConfirmation;
                    setReason("P%u charged: %lu ms", static_cast<unsigned int>(selectedPowerLevel), static_cast<unsigned long>(plannedChargeDurationMs));
                }
            }
            break;

        case PreparationState::AwaitingUnwindConfirmation:
            break;

        case PreparationState::AdjustingCharge:
            if (!chargeServoIsActive()) {
                plannedChargeDurationMs = static_cast<uint32_t>(static_cast<int32_t>(plannedChargeDurationMs) + pendingChargeCorrectionMs);
                pendingChargeCorrectionMs = 0;
                state = PreparationState::AwaitingUnwindConfirmation;
                setReason("Charge set: %lu ms", static_cast<unsigned long>(plannedChargeDurationMs));
            }
            break;

        case PreparationState::UnwindingCharge:
            if (!chargeServoIsActive()) {
                state = PreparationState::Ready;
                setReason("Ready");
                Serial.println("Cannon ready");
            }
            break;

        case PreparationState::Ready:
            if (!aimTrackingDataIsFresh(nowMs)) {
                setReason("Ready; waiting for aim");
            } else if (fabsf(aimState.latestErrorDegrees) > kAlignedToleranceDegrees) {
                setReason("Aiming: %.2f deg", static_cast<double>(aimState.latestErrorDegrees));
            } else {
                setReason("Ready");
            }
            break;

        case PreparationState::Idle:
        case PreparationState::Fault:
            break;
    }
}

void cancelCannonPreparation(const char *cancelReason) {
    if (state == PreparationState::Idle) {
        return;
    }

    stopAutomaticOutputs();
    state = PreparationState::Idle;
    fullyAutomaticSequence = false;
    aimLockCount = 0U;
    setReason("%s", cancelReason);
}

void cannonPreparationShotStarted() {
    if (state != PreparationState::Ready) {
        return;
    }

    cannonState.aimTrackingEnabled = false;
    applyAimServoAction(AimServoAction::Off);
    state = PreparationState::Idle;
    fullyAutomaticSequence = false;
    aimLockCount = 0U;
    setReason("Shot started");
}

} // namespace Hexapod
