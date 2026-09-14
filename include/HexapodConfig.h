#pragma once

#include <Arduino.h>

#include "RobotConfig.h"
#include "RobotTypes.h"

namespace Hexapod {
    // Wi-Fi
    constexpr char kAccessPointName[] = "Hexapod-Teams";
    constexpr char kAccessPointPassword[] = "Teams123";

    // Shared hardware
    constexpr uint32_t kPcaCheckIntervalMs = 1000;
    constexpr uint16_t kServoFrequencyHz = 50;
    constexpr size_t kMessageLogLineLength = 160;

    // Stand-up poses
    constexpr uint8_t kOriginalStandStartupPose[RobotConfig::kJointCount] = {
        90,
        180,
        170,
    };
    constexpr uint8_t kAll90StandingPose[RobotConfig::kJointCount] = {
        90,
        90,
        90,
    };
    constexpr uint8_t kWalkingTestPose[RobotConfig::kJointCount] = {
        90,
        150,
        140,
    };

    // Stand-up timing
    constexpr uint32_t kStandStartupPoseHoldMs = 1000;
    constexpr uint32_t kStandAllTo90TransitionMs = 5000;
    constexpr uint32_t kAll90ToWalkingPauseMs = 1000;
    constexpr uint32_t kWalkingPoseTransitionMs = 5000;
    constexpr uint32_t kStandServoUpdateIntervalMs = 20;

    // Gait leg order and diagonal pairs
    constexpr uint8_t kSingleWaveLegOrder[RobotConfig::kLegCount] = {
        static_cast<uint8_t>(LegId::LeftFront),
        static_cast<uint8_t>(LegId::LeftMiddle),
        static_cast<uint8_t>(LegId::LeftRear),
        static_cast<uint8_t>(LegId::RightRear),
        static_cast<uint8_t>(LegId::RightMiddle),
        static_cast<uint8_t>(LegId::RightFront),
    };
    constexpr uint8_t kDualRipplePairs[3][2] = {
        {static_cast<uint8_t>(LegId::LeftFront), static_cast<uint8_t>(LegId::RightRear)},
        {static_cast<uint8_t>(LegId::LeftMiddle), static_cast<uint8_t>(LegId::RightFront)},
        {static_cast<uint8_t>(LegId::LeftRear), static_cast<uint8_t>(LegId::RightMiddle)},
    };

    // Single-wave gait
    constexpr int kSingleWaveCoxaStepDegrees = 10;
    constexpr int kWaveFemurLiftOffsetDegrees = 27;
    constexpr int kWaveTibiaLiftOffsetDegrees = 10;
    constexpr uint32_t kWaveLiftDurationMs = 533;
    constexpr uint32_t kWaveSwingDurationMs = 400;
    constexpr uint32_t kWaveLowerDurationMs = 533;
    constexpr uint32_t kWavePushDurationMs = 1600;

    // Dual-ripple gait with grounded push
    constexpr int kDualRippleCoxaStepDegrees = 20;
    constexpr uint32_t kDualLiftDurationMs = 900;
    constexpr uint32_t kDualSwingDurationMs = 350;
    constexpr uint32_t kDualLowerDurationMs = 500;
    constexpr uint32_t kDualPushDurationMs = 3400;

    // Continuous ripple timing
    constexpr uint32_t kContinuousRippleLiftDurationMs = 700;
    constexpr uint32_t kContinuousRippleSwingDurationMs = 1200;
    constexpr uint32_t kContinuousRippleLowerDurationMs = 450;

    // V4 distributed-stance ripple
    constexpr int kContinuousRippleV4LeftCoxaStepDegrees = 12;
    constexpr int kContinuousRippleV4RightCoxaStepDegrees = 11;
    constexpr int kContinuousRippleV4LeftForwardAfterLiftDegrees = 8;
    constexpr int kContinuousRippleV4RightForwardAfterLiftDegrees = 7;
    constexpr int kContinuousRippleV4LeftRearAfterSwingDegrees = 9;
    constexpr int kContinuousRippleV4RightRearAfterSwingDegrees = 8;

    // V6 Cartesian-IK ripple
    constexpr int kContinuousRippleV6LeftCoxaStepDegrees = 15;
    constexpr int kContinuousRippleV6RightCoxaStepDegrees = 14;
    constexpr int kContinuousRippleV6LeftForwardAfterLiftDegrees = 10;
    constexpr int kContinuousRippleV6RightForwardAfterLiftDegrees = 9;
    constexpr int kContinuousRippleV6LeftRearAfterSwingDegrees = 11;
    constexpr int kContinuousRippleV6RightRearAfterSwingDegrees = 10;
    constexpr float kContinuousRippleCartesianLiftMm = 22.0F;
    constexpr float kIkTestSafeServoMinDeg = 4.0F;
    constexpr float kIkTestSafeServoMaxDeg = 176.0F;

    // Calibrated diagonal path planner
    constexpr int kForwardCoxaStepDegrees = 15;
    constexpr float kPathTargetForwardMm = 300.0F;
    constexpr float kPathTargetSideMm = 300.0F;
    constexpr float kMeasuredForwardMm = 130.0F;
    constexpr float kMeasuredForwardCycleCount = 3.0F;
    constexpr float kMeasuredTurnDegrees = 90.0F;
    constexpr float kMeasuredTurnCycleCount = 6.0F;

    // Turn fast turn profiles
    constexpr int kTurnCoxaStepDegrees = 15;
    constexpr uint8_t kTurn180CycleCount = 12;
    constexpr int kFastTurnCoxaStepDegrees = 22;
    constexpr uint8_t kFastTurn180CycleCount = 8;
    constexpr uint32_t kFastTurnLiftDurationMs = 450;
    constexpr uint32_t kFastTurnSwingDurationMs = 350;
    constexpr uint32_t kFastTurnLowerDurationMs = 450;
    constexpr uint32_t kFastTurnPushDurationMs = 1300;

    // Aim tracking
    constexpr float kAlignedToleranceDegrees = 3.0F;

    constexpr uint32_t kFreshMessageTimeoutMs = 2000;
    constexpr uint32_t kAimControlFreshTimeoutMs = 750;
    constexpr uint32_t kAimLockMaximumSampleGapMs = 500;

    constexpr uint8_t kAimServoChannel = 4;
    constexpr int kAimServoStopCommand = 90;
    constexpr int kAimServoMinimumSpeedOffset = 7;
    constexpr int kAimServoMaximumSpeedOffset = 30;
    constexpr float kAimServoSlowdownRangeDegrees = 25.0F;

    constexpr float kAimMicroPulseRangeDegrees = 15.0F;
    constexpr float kAimMicroPulseFineRangeDegrees = 6.0F;
    constexpr float kAimMicroPulseMediumRangeDegrees = 10.0F;
    constexpr uint32_t kAimMicroPulseFineDurationMs = 20;
    constexpr uint32_t kAimMicroPulseMediumDurationMs = 30;
    constexpr uint32_t kAimMicroPulseCoarseDurationMs = 45;
    constexpr uint32_t kAimMicroPulsePauseMs = 160;
    constexpr uint8_t kAimMicroPulseRequiredFrames = 2;

    constexpr int kManualAimSpeedOffset = 15;
    constexpr uint32_t kManualAimRunDurationMs = 250;

    // Cannon shoot servo
    constexpr uint8_t kShootServoChannel = 7;
    constexpr int kShootForwardCommand = 60;
    constexpr int kShootResetCommand = 120;
    constexpr uint32_t kShootPhaseDurationMs = 500;

    // Cannon tilt servo
    constexpr uint8_t kTiltServoChannel = 5;
    constexpr int kTiltUpCommand = 100;
    constexpr int kTiltDownCommand = 80;
    constexpr uint32_t kTiltRunDurationMs = 200;

    // Cannon charge servo
    constexpr uint8_t kChargeServoChannel = 6;
    constexpr int kChargeCommand = 120;
    constexpr int kUnchargeCommand = 60;
    constexpr uint32_t kChargeRunDurationMs = 1000;
    constexpr uint32_t kChargeFineStepDurationMs = 500;
    constexpr uint32_t kCannonPreparationChargeAdjustmentMs = 200;
    constexpr uint32_t kRecordedUnwindExtraDurationMs = 500;
    constexpr uint32_t kChargeP1DurationMs = 4000;
    constexpr uint32_t kChargeP2DurationMs = 6000;
    constexpr uint32_t kChargeP3DurationMs = 8000;
    constexpr uint32_t kMaximumChargeDurationMs = 10000;
    constexpr uint32_t kMaximumUnchargeDurationMs = kMaximumChargeDurationMs + kRecordedUnwindExtraDurationMs;

    // Automatic cannon preparation
    constexpr uint8_t kCannonAimLockSamples = 5;
    constexpr uint32_t kCannonTiltHomeDurationMs = 1200;

    //Calculated shot settings
    struct ShotCalibrationPoint {
        float representativeRangeCm;
        float observedMinimumCm;
        float observedMaximumCm;
        uint16_t tiltUpMs;
        uint8_t powerLevel;
    };

    constexpr ShotCalibrationPoint kShotCalibration[] = {
        {65.0F, 60.0F, 70.0F, 0, 2},
        {117.5F, 115.0F, 120.0F, 200, 2},
        {135.0F, 130.0F, 140.0F, 400, 2},
        {145.0F, 140.0F, 150.0F, 600, 2},
        {150.0F, 150.0F, 150.0F, 800, 2},
    };

    constexpr float kMinimumCalibratedRangeCm = 60.0F;
    constexpr float kMaximumCalibratedRangeCm = 150.0F;
} // namespace Hexapod
