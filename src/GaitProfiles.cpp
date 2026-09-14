#include <Arduino.h>
#include <stdio.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "GaitMath.h"
#include "Kinematics.h"

namespace Hexapod {

bool isContinuousRippleMode(WalkMode mode) {
    return mode == WalkMode::ContinuousRippleV4DistributedStance || mode == WalkMode::ContinuousRippleV6FullIk || mode == WalkMode::ContinuousRippleV6ObstacleAvoidance;
}

bool continuousRippleModeActive() {
    return isContinuousRippleMode(gaitState.walkMode);
}

bool continuousRippleDistributedStanceActive() {
    return gaitState.walkMode == WalkMode::ContinuousRippleV4DistributedStance || gaitState.walkMode == WalkMode::ContinuousRippleV6FullIk || gaitState.walkMode == WalkMode::ContinuousRippleV6ObstacleAvoidance;
}

bool continuousRippleFullIkActive() {
    return gaitState.walkMode == WalkMode::ContinuousRippleV6FullIk || gaitState.walkMode == WalkMode::ContinuousRippleV6ObstacleAvoidance;
}

int mirroredCoxaAngle(uint8_t legIndex, int stepDegrees, bool forward) {
    const bool leftSide = legIndex <= static_cast<uint8_t>(LegId::LeftRear);
    const int direction = leftSide ? -1 : 1;
    return kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)] + direction * (forward ? stepDegrees : -stepDegrees);
}

bool calculatedPathModeActive() {
    return gaitState.walkMode == WalkMode::CalculatedPath;
}

bool calculatedPathTurnActive() {
    return calculatedPathModeActive() && gaitState.calculatedPathPhase == CalculatedPathPhase::TurnToTarget;
}

int walkCoxaStepDegrees() {
    if (gaitState.walkMode == WalkMode::DualRipple) {
        return kDualRippleCoxaStepDegrees;
    }

    if (calculatedPathModeActive() && gaitState.calculatedPathPhase == CalculatedPathPhase::MoveToTarget) {
        return gaitState.completedCalculatedPathPhaseCycles + 1U >= pathPlanState.forwardCycles ? pathPlanState.finalCoxaStepDegrees : kForwardCoxaStepDegrees;
    }

    return kSingleWaveCoxaStepDegrees;
}

bool turnModeActive() {
    return gaitState.walkMode == WalkMode::TurnClockwise || gaitState.walkMode == WalkMode::TurnCounterClockwise || gaitState.walkMode == WalkMode::TurnClockwiseFast ||
           gaitState.walkMode == WalkMode::TurnCounterClockwiseFast || calculatedPathTurnActive();
}

bool fastTurnModeActive() {
    return gaitState.walkMode == WalkMode::TurnClockwiseFast || gaitState.walkMode == WalkMode::TurnCounterClockwiseFast;
}

bool clockwiseTurnActive() {
    return gaitState.walkMode == WalkMode::TurnClockwise || gaitState.walkMode == WalkMode::TurnClockwiseFast || (calculatedPathTurnActive() && pathPlanState.headingDegrees > 0.0F);
}

int turnCoxaStepDegrees() {
    return fastTurnModeActive() ? kFastTurnCoxaStepDegrees : kTurnCoxaStepDegrees;
}

uint8_t turnCycleCount() {
    return fastTurnModeActive() ? kFastTurn180CycleCount : kTurn180CycleCount;
}

bool pairedLegMovementActive() {
    return gaitState.walkMode == WalkMode::DualRipple || continuousRippleModeActive() || calculatedPathModeActive() || turnModeActive();
}

int walkForwardCoxaAngle(uint8_t legIndex) {
    if (turnModeActive()) {
        const int direction = clockwiseTurnActive() ? -1 : 1;
        return kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)] + direction * turnCoxaStepDegrees();
    }

    return mirroredCoxaAngle(legIndex, walkCoxaStepDegrees(), true);
}

int walkRearCoxaAngle(uint8_t legIndex) {
    if (turnModeActive()) {
        const int direction = clockwiseTurnActive() ? 1 : -1;
        return kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)] + direction * turnCoxaStepDegrees();
    }

    return mirroredCoxaAngle(legIndex, walkCoxaStepDegrees(), false);
}

uint8_t continuousRipplePairForLeg(uint8_t legIndex) {
    for (uint8_t pairIndex = 0; pairIndex < 3U; ++pairIndex) {
        if (legIndex == kDualRipplePairs[pairIndex][0] || legIndex == kDualRipplePairs[pairIndex][1]) {
            return pairIndex;
        }
    }

    return 0;
}

int continuousRippleStepForLeg(uint8_t legIndex) {
    const bool leftSide = legIndex <= static_cast<uint8_t>(LegId::LeftRear);
    if (continuousRippleFullIkActive()) {
        return leftSide ? kContinuousRippleV6LeftCoxaStepDegrees : kContinuousRippleV6RightCoxaStepDegrees;
    }

    if (!continuousRippleDistributedStanceActive()) {
        return walkCoxaStepDegrees();
    }
    return leftSide ? kContinuousRippleV4LeftCoxaStepDegrees : kContinuousRippleV4RightCoxaStepDegrees;
}

int continuousRippleCoxaTargetAfterPair(uint8_t legIndex, uint8_t completedPairIndex) {
    const uint8_t legPair = continuousRipplePairForLeg(legIndex);
    const int stepDegrees = continuousRippleStepForLeg(legIndex);
    if (legPair == completedPairIndex) {
        return mirroredCoxaAngle(legIndex, stepDegrees, true);
    }

    if (legPair == static_cast<uint8_t>((completedPairIndex + 1U) % 3U)) {
        return mirroredCoxaAngle(legIndex, stepDegrees, false);
    }

    return kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)];
}

int continuousRipplePrimingCoxaTarget(uint8_t legIndex) {
    const uint8_t legPair = continuousRipplePairForLeg(legIndex);
    const int stepDegrees = continuousRippleStepForLeg(legIndex);
    if (legPair == 0U) {
        return mirroredCoxaAngle(legIndex, stepDegrees, false);
    }

    if (gaitState.ripplePrimeStep > 0U && legPair == 2U) {
        return mirroredCoxaAngle(legIndex, stepDegrees, true);
    }

    return kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)];
}

bool allLegCommandsMatchWalkingPose() {
    if (!allLegCommandsKnown()) {
        return false;
    }

    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        if (standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Coxa)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)] ||
            standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Femur)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Femur)] ||
            standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Tibia)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Tibia)]) {
            return false;
        }
    }
    return true;
}

bool allLegCommandsMatchWaveRearPose() {
    if (!allLegCommandsKnown()) {
        return false;
    }

    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        if (standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Coxa)] != walkRearCoxaAngle(legIndex) ||
            standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Femur)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Femur)] ||
            standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Tibia)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Tibia)]) {
            return false;
        }
    }
    return true;
}

bool allLegCommandsMatchContinuousRippleStopPose() {
    if (!allLegCommandsKnown() || !continuousRippleModeActive()) {
        return false;
    }

    const uint8_t completedPairIndex = static_cast<uint8_t>((gaitState.dualRipplePairIndex + 2U) % 3U);
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        const int expectedCoxa = gaitState.ripplePriming ? continuousRipplePrimingCoxaTarget(legIndex) : continuousRippleCoxaTargetAfterPair(legIndex, completedPairIndex);
        if (standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Coxa)] != expectedCoxa ||
            standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Femur)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Femur)] ||
            standState.legCommandDegrees[legIndex][static_cast<uint8_t>(JointId::Tibia)] != kWalkingTestPose[static_cast<uint8_t>(JointId::Tibia)]) {
            return false;
        }
    }
    return true;
}

bool calculateCartesianLiftServo(uint8_t legIndex, float coxaServoDeg, float liftFraction, ServoAnglesDeg &result, char *fault, size_t faultSize) {
    result = {0.0F, 0.0F, 0.0F, false};
    const ServoAnglesDeg groundServo = {
        coxaServoDeg,
        static_cast<float>(kWalkingTestPose[static_cast<uint8_t>(JointId::Femur)]),
        static_cast<float>(kWalkingTestPose[static_cast<uint8_t>(JointId::Tibia)]),
        true,
    };

    Kinematics kinematics;
    const LegId leg = static_cast<LegId>(legIndex);
    const JointAnglesDeg groundModel = kinematics.servoToModelAngles(leg, groundServo, fault, faultSize);
    const FootTarget groundFoot = groundModel.valid ? kinematics.solveForwardLeg(leg, groundModel, fault, faultSize) : FootTarget{{0.0F, 0.0F, 0.0F}, false};
    if (!groundFoot.valid) {
        return false;
    }

    Vec3 target = groundFoot.bodyPositionMm;
    target.z += kContinuousRippleCartesianLiftMm * fminf(1.0F, fmaxf(0.0F, liftFraction));

    const JointAnglesDeg model = kinematics.solveLeg( leg, target, fault, faultSize);
    result = model.valid ? kinematics.modelToServoAngles(leg, model, fault, faultSize) : ServoAnglesDeg{0.0F, 0.0F, 0.0F, false};

    if (!servoInsideIkTestMargin(result) || fabsf(result.coxa - coxaServoDeg) > 0.05F) {
        snprintf(fault, faultSize, "Cartesian IK unsafe leg %u at %.2f lift", static_cast<unsigned>(legIndex), static_cast<double>(liftFraction));
        result.valid = false;
        return false;
    }

    return true;
}

bool validateContinuousRippleV6FullIk() {
    char detail[96] = "";
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        const bool leftSide = legIndex <= static_cast<uint8_t>(LegId::LeftRear);
        const int step = leftSide ? kContinuousRippleV6LeftCoxaStepDegrees : kContinuousRippleV6RightCoxaStepDegrees;
        const float coxaSamples[3] = {
            static_cast<float>(90 - step),  //rear
            90.0F,                          //center
            static_cast<float>(90 + step),  //forward
        };

        // Ali se lahko noga gladko dvigne in spusti v rear, center in forward položaju?
        for (float coxa : coxaSamples) {
            for (uint16_t sample = 0; sample <= 100U; ++sample) {
                ServoAnglesDeg servo = {};
                if (!calculateCartesianLiftServo(legIndex, coxa, static_cast<float>(sample) / 100.0F, servo, detail, sizeof(detail))) {
                    snprintf(standState.fault, sizeof(standState.fault), "V6 lift preflight failed: %s", detail);
                    return false;
                }
            }
        }

        // Ali lahko coxa naredi celoten rear-to-forward lok, ko je stopalo na tleh ali na maksimalni višini?
        for (uint16_t sample = 0; sample <= 100U; ++sample) {
            const float coxa = static_cast<float>(90 - step) + static_cast<float>(2 * step) * static_cast<float>(sample) / 100.0F;
            for (float liftFraction : {0.0F, 1.0F}) {
                ServoAnglesDeg servo = {};
                if (!calculateCartesianLiftServo(legIndex, coxa, liftFraction, servo, detail, sizeof(detail))) {
                    snprintf(standState.fault, sizeof(standState.fault), "V6 arc preflight failed: %s", detail);
                    return false;
                }
                if (fabsf(servo.coxa - coxa) > 0.05F) {
                    snprintf(standState.fault, sizeof(standState.fault), "V6 coxa round-trip failed leg %u", static_cast<unsigned>(legIndex));
                    return false;
                }
            }
        }
    }

    standState.fault[0] = '\0';
    return true;
}

//pair 0 = LF + RR
//pair 1 = LM + RF
//pair 2 = LR + RM
int continuousRippleDistributedCoxaTarget(StandStage stage, uint8_t legIndex) {
    const uint8_t legPair = continuousRipplePairForLeg(legIndex);
    const uint8_t activePair = gaitState.dualRipplePairIndex;
    const uint8_t previousPair = static_cast<uint8_t>((activePair + 2U) % 3U);
    const uint8_t nextPair = static_cast<uint8_t>((activePair + 1U) % 3U);

    const bool leftSide = legIndex <= static_cast<uint8_t>(LegId::LeftRear);
    const int fullStepDegrees = continuousRippleStepForLeg(legIndex);
    const int forwardAfterLiftDegrees = continuousRippleFullIkActive() ? (leftSide ? kContinuousRippleV6LeftForwardAfterLiftDegrees : kContinuousRippleV6RightForwardAfterLiftDegrees) : (leftSide
               ? kContinuousRippleV4LeftForwardAfterLiftDegrees : kContinuousRippleV4RightForwardAfterLiftDegrees);
    const int rearAfterSwingDegrees = continuousRippleFullIkActive() ? (leftSide ? kContinuousRippleV6LeftRearAfterSwingDegrees : kContinuousRippleV6RightRearAfterSwingDegrees) : (leftSide ? kContinuousRippleV4LeftRearAfterSwingDegrees
               : kContinuousRippleV4RightRearAfterSwingDegrees);

    if (stage == StandStage::WaveLift) {
        if (legPair == previousPair) {
            return mirroredCoxaAngle( legIndex, forwardAfterLiftDegrees, true);
        }
    } else if (stage == StandStage::WaveSwingForward) {
        if (legPair == activePair) {
            return mirroredCoxaAngle( legIndex, fullStepDegrees, true);
        }

        if (legPair == previousPair) {
            return kWalkingTestPose[static_cast<uint8_t>(JointId::Coxa)];
        }

        if (legPair == nextPair) {
            return mirroredCoxaAngle( legIndex, rearAfterSwingDegrees, false);
        }
    } else if (stage == StandStage::WaveLower && legPair == nextPair) {
        return mirroredCoxaAngle( legIndex, fullStepDegrees, false);
    }

    return static_cast<int>(lroundf(gaitState.waveStageStartAngles[legIndex][static_cast<uint8_t>(JointId::Coxa)]));
}

int waveTargetAngle(StandStage stage, uint8_t legIndex, uint8_t jointIndex) {
    const bool activeLeg = activeWalkLeg(legIndex);
    const bool lifted = (stage == StandStage::WaveLift || stage == StandStage::WaveSwingForward || stage == StandStage::WaveRecoveryLift || stage == StandStage::WaveRecoverySwing) && (!pairedLegMovementActive() || activeLeg);

    if (continuousRippleFullIkActive() && activeLeg && stage == StandStage::WaveSwingForward && (jointIndex == static_cast<uint8_t>(JointId::Femur) || jointIndex == static_cast<uint8_t>(JointId::Tibia))) {
        return static_cast<int>(lroundf( gaitState.waveStageStartAngles[legIndex][jointIndex]));
    }

    if (jointIndex == static_cast<uint8_t>(JointId::Tibia)) {
        return kWalkingTestPose[jointIndex] + (lifted ? kWaveTibiaLiftOffsetDegrees : 0);
    }

    if (jointIndex == static_cast<uint8_t>(JointId::Femur)) {
        return kWalkingTestPose[jointIndex] + (lifted ? kWaveFemurLiftOffsetDegrees : 0);
    }

    if (jointIndex == static_cast<uint8_t>(JointId::Coxa) && continuousRippleDistributedStanceActive() && !gaitState.ripplePriming && (stage == StandStage::WaveLift || stage == StandStage::WaveSwingForward ||
         stage == StandStage::WaveLower)) {
        return continuousRippleDistributedCoxaTarget(stage, legIndex);
    }

    if (pairedLegMovementActive() && (stage == StandStage::WaveLift || stage == StandStage::WaveRecoveryLift)) {
        return gaitState.waveStageStartAngles[legIndex][jointIndex];
    }

    if (continuousRippleModeActive() && stage == StandStage::WaveSwingForward) {
        return gaitState.ripplePriming ? continuousRipplePrimingCoxaTarget(legIndex) : continuousRippleCoxaTargetAfterPair(legIndex, gaitState.dualRipplePairIndex);
    }

    if (continuousRippleModeActive() && stage == StandStage::WaveLower) {
        return gaitState.waveStageStartAngles[legIndex][jointIndex];
    }

    if (stage == StandStage::WaveSwingForward || stage == StandStage::WaveLower || stage == StandStage::WaveRecoverySwing || stage == StandStage::WaveRecoveryLower) {
        return walkForwardCoxaAngle(legIndex);
    }

    if (stage == StandStage::WavePush) {
        return walkRearCoxaAngle(legIndex);
    }

    if (stage == StandStage::WaveRecoveryLift) {
        return walkRearCoxaAngle(legIndex);
    }

    return kWalkingTestPose[jointIndex];
}

} // namespace Hexapod
