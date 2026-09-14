#pragma once

#include "HexapodConfig.h"

namespace Hexapod {
    bool waveStageIsActive();
    bool legOperationIsActive();
    const char *standModeName();
    const char *standStageName();
    void setStandFault(const char *message);
    bool writeLegJointPrecise(uint8_t legIndex, uint8_t jointIndex, float angleDeg);
    bool allLegCommandsKnown();
    int mirroredCoxaAngle(uint8_t legIndex, int stepDegrees, bool forward);
    bool calculatedPathModeActive();
    bool calculatedPathTurnActive();
    bool isContinuousRippleMode(WalkMode mode);
    bool continuousRippleModeActive();
    bool continuousRippleDistributedStanceActive();
    bool continuousRippleFullIkActive();
    int walkCoxaStepDegrees();
    bool turnModeActive();
    bool fastTurnModeActive();
    bool clockwiseTurnActive();
    int turnCoxaStepDegrees();
    uint8_t turnCycleCount();
    bool pairedLegMovementActive();
    int walkForwardCoxaAngle(uint8_t legIndex);
    int walkRearCoxaAngle(uint8_t legIndex);
    uint8_t continuousRipplePairForLeg(uint8_t legIndex);
    int continuousRippleCoxaTargetAfterPair(uint8_t legIndex, uint8_t completedPairIndex);
    int continuousRipplePrimingCoxaTarget(uint8_t legIndex);
    bool allLegCommandsMatchWalkingPose();
    bool allLegCommandsMatchWaveRearPose();
    bool allLegCommandsMatchContinuousRippleStopPose();
    bool calculateCartesianLiftServo(uint8_t legIndex, float coxaServoDeg, float liftFraction, ServoAnglesDeg &result,
                                     char *fault, size_t faultSize);
    bool validateContinuousRippleV6FullIk();
    bool writeLegPose(uint8_t legIndex, const float pose[RobotConfig::kJointCount]);
    bool writeActiveUniformStandPose(const float pose[RobotConfig::kJointCount]);
    void disableAllLegOutputs();
    void startStandupSequence();
    bool startWalkingPoseSequence();
    bool waveStageMovesAllLegs(StandStage stage);
    bool activeWalkLeg(uint8_t legIndex);
    void resetActiveWalkGroup();
    void advanceContinuousRipplePriming();
    bool advanceActiveWalkGroup();
    void beginWaveStage(StandStage stage, uint32_t nowMs);
    int waveTargetAngle(StandStage stage, uint8_t legIndex, uint8_t jointIndex);
    bool updateWaveSequence(uint32_t nowMs);
    void updateLegOperation();
} // namespace Hexapod
