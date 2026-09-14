#include <Arduino.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "GaitMath.h"

namespace Hexapod {

bool waveStageIsActive() {
    return standState.stage == StandStage::WaveLift || standState.stage == StandStage::WaveSwingForward || standState.stage == StandStage::WaveLower || standState.stage == StandStage::WavePush ||
           standState.stage == StandStage::WaveRecoveryLift || standState.stage == StandStage::WaveRecoverySwing || standState.stage == StandStage::WaveRecoveryLower;
}

bool waveStageMovesAllLegs(StandStage stage) {
    return stage == StandStage::WavePush || (continuousRippleDistributedStanceActive() && !gaitState.ripplePriming && (stage == StandStage::WaveLift || stage == StandStage::WaveSwingForward || stage == StandStage::WaveLower)) ||
           (continuousRippleModeActive() && !gaitState.ripplePriming && stage == StandStage::WaveSwingForward);
}

bool activeWalkLeg(uint8_t legIndex) {
    if (pairedLegMovementActive()) {
        return legIndex == kDualRipplePairs[gaitState.dualRipplePairIndex][0] || legIndex == kDualRipplePairs[gaitState.dualRipplePairIndex][1];
    }

    return legIndex == kSingleWaveLegOrder[gaitState.waveLegOrderIndex];
}

void resetActiveWalkGroup() {
    gaitState.waveLegOrderIndex = 0;
    gaitState.dualRipplePairIndex = 0;
}

void advanceContinuousRipplePriming() {
    if (gaitState.ripplePrimeStep == 0U) {
        gaitState.ripplePrimeStep = 1U;
        gaitState.dualRipplePairIndex = 2U;
    } else if (gaitState.rippleFullPriming && gaitState.ripplePrimeStep == 1U) {
        gaitState.ripplePrimeStep = 2U;
        gaitState.dualRipplePairIndex = 1U;
    } else {
        gaitState.ripplePriming = false;
        gaitState.rippleFullPriming = false;
        gaitState.ripplePrimeStep = 0U;
        gaitState.dualRipplePairIndex = 0U;
    }
}

bool advanceActiveWalkGroup() {
    if (pairedLegMovementActive()) {
        ++gaitState.dualRipplePairIndex;
        return gaitState.dualRipplePairIndex < 3U;
    }

    ++gaitState.waveLegOrderIndex;

    return gaitState.waveLegOrderIndex < RobotConfig::kLegCount;
}

void beginWaveStage(StandStage stage, uint32_t nowMs) {
    standState.stage = stage;
    standState.stageStartedAtMs = nowMs;
    standState.lastServoUpdateAtMs = nowMs;
    standState.progress = 0.0F;

    const bool allLegs = waveStageMovesAllLegs(stage);
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        if (!allLegs && !activeWalkLeg(legIndex)) {
            continue;
        }

        for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
            gaitState.waveStageStartAngles[legIndex][jointIndex] = standState.legCommandDegrees[legIndex][jointIndex];
        }
    }
}

bool updateWaveSequence(uint32_t nowMs) {
    if (!waveStageIsActive()) {
        return false;
    }

    if (nowMs - standState.lastServoUpdateAtMs < kStandServoUpdateIntervalMs) {
        return true;
    }

    standState.lastServoUpdateAtMs += kStandServoUpdateIntervalMs;
    if (nowMs - standState.lastServoUpdateAtMs >= kStandServoUpdateIntervalMs) {
        standState.lastServoUpdateAtMs = nowMs;
    }

    const bool continuousRippleTiming = continuousRippleModeActive();
    const bool fastTurnTiming = fastTurnModeActive();
    const bool usesDualTiming = gaitState.walkMode == WalkMode::DualRipple || continuousRippleTiming || (calculatedPathModeActive() && !calculatedPathTurnActive());

    uint32_t durationMs;
    if (fastTurnTiming) {
        durationMs = kFastTurnLiftDurationMs;
    } else if (continuousRippleTiming) {
        durationMs = kContinuousRippleLiftDurationMs;
    } else if (usesDualTiming) {
        durationMs = kDualLiftDurationMs;
    } else {
        durationMs = kWaveLiftDurationMs;
    }

    if (standState.stage == StandStage::WaveSwingForward || standState.stage == StandStage::WaveRecoverySwing) {
        durationMs = fastTurnTiming ? kFastTurnSwingDurationMs : continuousRippleTiming ? kContinuousRippleSwingDurationMs : usesDualTiming ? kDualSwingDurationMs : kWaveSwingDurationMs;
    } else if (standState.stage == StandStage::WaveLower || standState.stage == StandStage::WaveRecoveryLower) {
        durationMs = fastTurnTiming ? kFastTurnLowerDurationMs : continuousRippleTiming ? kContinuousRippleLowerDurationMs : usesDualTiming ? kDualLowerDurationMs : kWaveLowerDurationMs;
    } else if (standState.stage == StandStage::WavePush) {
        durationMs = fastTurnTiming ? kFastTurnPushDurationMs : usesDualTiming ? kDualPushDurationMs : kWavePushDurationMs;
    }

    standState.progress = fminf( 1.0F, static_cast<float>(nowMs - standState.stageStartedAtMs) / static_cast<float>(durationMs));
    const float eased = continuousRippleTiming ? quinticSmoothStep(standState.progress) : standState.progress * standState.progress * (3.0F - 2.0F * standState.progress);
    const bool allLegs = waveStageMovesAllLegs(standState.stage);
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        if (!allLegs && !activeWalkLeg(legIndex)) {
            continue;
        }

        const bool activeLeg = activeWalkLeg(legIndex);
        const bool verticalStage = activeLeg && (standState.stage == StandStage::WaveLift || standState.stage == StandStage::WaveLower);
        const float coxaStart = gaitState.waveStageStartAngles[legIndex][static_cast<uint8_t>(JointId::Coxa)];
        const float coxaTarget = static_cast<float>(waveTargetAngle(standState.stage, legIndex, static_cast<uint8_t>(JointId::Coxa)));
        const bool coxaMoves = fabsf(coxaTarget - coxaStart) >= 0.001F;

        // V6
        const bool v6CartesianLeg = continuousRippleFullIkActive() && (verticalStage || coxaMoves);
        ServoAnglesDeg v6Servo = {};
        if (v6CartesianLeg) {
            float liftFraction = 0.0F;

            if (activeLeg) {
                liftFraction = standState.stage == StandStage::WaveLift ? eased :
                        standState.stage == StandStage::WaveLower ? 1.0F - eased :
                        standState.stage == StandStage::WaveSwingForward ? 1.0F : 0.0F;
            }

            char fault[96] = "";
            if (!calculateCartesianLiftServo(legIndex, coxaStart + (coxaTarget - coxaStart) * eased, liftFraction, v6Servo, fault, sizeof(fault))) {
                setStandFault(fault);
                return true;
            }
        }

        for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
            if (v6CartesianLeg) {
                /*
                * Zapiši coxa samo če se coxa premika
                * Zapiši femur/tibia samo če aktivna noga dela Lift ali Lower
                */

                if (!(jointIndex == static_cast<uint8_t>(JointId::Coxa) && coxaMoves) && !(verticalStage && (jointIndex == static_cast<uint8_t>(JointId::Femur) || jointIndex == static_cast<uint8_t>(JointId::Tibia)))) {
                    continue;
                }

                const float angle = jointIndex == static_cast<uint8_t>(JointId::Coxa) ? v6Servo.coxa : jointIndex == static_cast<uint8_t>(JointId::Femur) ? v6Servo.femur : v6Servo.tibia;
                if (!writeLegJointPrecise(legIndex, jointIndex, angle)) {
                    return true;
                }

                continue;
            }

            //Ne v6
            const float start = gaitState.waveStageStartAngles[legIndex][jointIndex];
            const float target = static_cast<float>( waveTargetAngle(standState.stage, legIndex, jointIndex));
            if (fabsf(target - start) < 0.001F) {
                continue;
            }

            const float angle = start + (target - start) * eased;
            if (!writeLegJointPrecise(legIndex, jointIndex, angle)) {
                return true;
            }
        }
    }

    if (standState.progress < 1.0F) {
        return true;
    }

    switch (standState.stage) {
        case StandStage::WaveLift:
            beginWaveStage(StandStage::WaveSwingForward, nowMs);
            break;

        case StandStage::WaveSwingForward:
            beginWaveStage(StandStage::WaveLower, nowMs);
            break;

        case StandStage::WaveLower:
            if (continuousRippleModeActive()) {
                if (!gaitState.ripplePriming) {
                    gaitState.dualRipplePairIndex = static_cast<uint8_t>( (gaitState.dualRipplePairIndex + 1U) % 3U);
                }

                // Safe stop
                if (gaitState.waveStopRequested) {
                    standState.stage = StandStage::WaveStopped;
                    standState.progress = 1.0F;
                } else {
                    if (gaitState.ripplePriming) {
                        advanceContinuousRipplePriming();
                    }

                    beginWaveStage(StandStage::WaveLift, nowMs);
                }
            } else if (advanceActiveWalkGroup()) {
                beginWaveStage(StandStage::WaveLift, nowMs);
                // Last leg/pair finished -> push
            } else {
                resetActiveWalkGroup();
                beginWaveStage(StandStage::WavePush, nowMs);
            }
            break;

        case StandStage::WavePush: {
            // Safe stop
            if (gaitState.waveStopRequested) {
                standState.stage = StandStage::WaveStopped;
                standState.progress = 1.0F;
                Serial.println("Gait stopped");
                break;
            }

            bool finiteTestComplete = false;
            const char *completionMessage = "Continuous gait stopped safely. ";

            // Calculated path
            if (calculatedPathModeActive()) {
                ++gaitState.completedCalculatedPathPhaseCycles;
                const bool turning = gaitState.calculatedPathPhase == CalculatedPathPhase::TurnToTarget;
                const uint8_t phaseTarget = turning ? pathPlanState.turnCycles : pathPlanState.forwardCycles;

                if (gaitState.completedCalculatedPathPhaseCycles >= phaseTarget) {
                    gaitState.completedCalculatedPathPhaseCycles = 0;
                    resetActiveWalkGroup();

                    if (turning) {
                        gaitState.calculatedPathPhase = CalculatedPathPhase::MoveToTarget;
                        beginWaveStage(StandStage::WaveRecoveryLift, nowMs);
                        break;
                    }

                    finiteTestComplete = true;
                    completionMessage = pathPlanState.targetRightMm < 0.0F ? "Forward-left complete" : "Forward-right complete";
                }
                // Turning
            } else if (turnModeActive()) {
                ++gaitState.completedTurnCycles;
                const uint8_t requiredCycles = turnCycleCount();

                finiteTestComplete = gaitState.completedTurnCycles >= requiredCycles;
                if (finiteTestComplete) {
                    completionMessage = fastTurnModeActive() ? "Fast turn complete" : "Turn complete";
                }
            }

            if (finiteTestComplete) {
                standState.stage = StandStage::WaveStopped;
                standState.progress = 1.0F;
                Serial.println(completionMessage);
            } else {
                resetActiveWalkGroup();
                beginWaveStage(StandStage::WaveRecoveryLift, nowMs);
            }

            break;
        }

        case StandStage::WaveRecoveryLift:
            beginWaveStage(StandStage::WaveRecoverySwing, nowMs);
            break;

        case StandStage::WaveRecoverySwing:
            beginWaveStage(StandStage::WaveRecoveryLower, nowMs);
            break;

        case StandStage::WaveRecoveryLower:
            if (advanceActiveWalkGroup()) {
                beginWaveStage(StandStage::WaveRecoveryLift, nowMs);
            } else {
                resetActiveWalkGroup();
                beginWaveStage(StandStage::WavePush, nowMs);
            }
            break;

        default:
            break;
    }
    return true;
}

} // namespace Hexapod
