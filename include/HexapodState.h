#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>

#include "HexapodConfig.h"

namespace Hexapod {
    // Current camera data
    struct AimState {
        bool hasMessage;
        float latestErrorDegrees;
        bool hasTargetDistance;
        float targetDistanceCm;
        uint32_t targetDistanceAtMs;
        uint32_t latestSequence;
        uint32_t acceptedMessageCount;
        uint32_t lastMessageAtMs;
        bool tagsVisible;
        bool cannonTagDetected;
        bool targetTagDetected;
        uint32_t latestCameraFrameSequence;
    };

    struct PcaState {
        bool ready;
        bool checked;
        bool solderedReady;
        uint32_t lastCheckAtMs;
    };

    struct CannonState {
        bool aimTrackingEnabled;
        AimServoAction appliedAimAction = AimServoAction::Off;
        int appliedAimCommand = -1;
        bool manualAimActive;
        AimServoAction manualAimAction = AimServoAction::Stop;
        uint32_t manualAimStopAtMs;
        ShootPhase shootPhase = ShootPhase::Idle;
        bool shootReturnAfterForward;
        uint32_t shootStopAtMs;
        TiltAction tiltAction = TiltAction::Idle;
        uint32_t tiltStopAtMs;
        ChargeAction chargeAction = ChargeAction::Idle;
        uint32_t chargeStopAtMs;
        uint32_t chargeStartedAtMs;
        uint32_t chargePlannedDurationMs;
        uint32_t estimatedWoundDurationMs;
    };

    // Hexapod state
    struct StandState {
        StandStage stage = StandStage::Idle; //which stage is currently active
        StandMode mode = StandMode::None;
        uint32_t stageStartedAtMs;
        uint32_t lastServoUpdateAtMs;
        float progress;
        char fault[96];
        int legCommandDegrees[RobotConfig::kLegCount][RobotConfig::kJointCount];
        bool legCommandKnown[RobotConfig::kLegCount][RobotConfig::kJointCount];
    };

    // Runtime state for the active leg gait
    struct GaitState {
        uint8_t waveLegOrderIndex;
        uint8_t dualRipplePairIndex;
        uint8_t completedTurnCycles;
        uint8_t completedCalculatedPathPhaseCycles;
        CalculatedPathPhase calculatedPathPhase = CalculatedPathPhase::None;
        WalkMode walkMode = WalkMode::None; //which gate is active
        bool waveStopRequested;
        bool ripplePriming;
        bool rippleFullPriming; //after turning
        uint8_t ripplePrimeStep;
        int waveStageStartAngles[RobotConfig::kLegCount][RobotConfig::kJointCount];
    };

    // Calculated plan
    struct PathPlanState {
        uint8_t turnCycles;
        uint8_t forwardCycles;
        int finalCoxaStepDegrees = kForwardCoxaStepDegrees;
        float targetForwardMm;
        float targetRightMm;
        float headingDegrees;
        float distanceMm;
        float predictedForwardMm;
        float predictedRightMm;
    };

    extern WebServer webServer;
    extern Adafruit_PWMServoDriver aimPwm;
    extern Adafruit_PWMServoDriver solderedPwm;
    extern AimState aimState;
    extern PcaState pcaState;
    extern CannonState cannonState;
    extern StandState standState;
    extern GaitState gaitState;
    extern PathPlanState pathPlanState;
} // namespace Hexapod
