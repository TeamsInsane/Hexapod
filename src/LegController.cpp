#include <Arduino.h>
#include <stdio.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "ObstacleNavigation.h"

namespace Hexapod {

bool legOperationIsActive() {
    return standState.stage == StandStage::StartupPoseHold || standState.stage == StandStage::MovingAllTo90
            || standState.stage == StandStage::All90Pause || standState.stage == StandStage::MovingToWalkingPose
            || waveStageIsActive() || obstacleNavigationActive();
}

const char *standModeName() {
    switch (standState.mode) {
        case StandMode::None:
            return "none";
        case StandMode::FullRobot:
            return "full robot";
    }

    return "none";
}

const char *standStageName() {
    switch (standState.stage) {
        case StandStage::Idle:
            return "idle";
        case StandStage::StartupPoseHold:
        case StandStage::MovingAllTo90:
        case StandStage::All90Pause:
        case StandStage::MovingToWalkingPose:
            return "standing up";
        case StandStage::WalkingPoseComplete:
            return "ready to walk";
        case StandStage::WaveLift:
        case StandStage::WaveSwingForward:
        case StandStage::WaveLower:
        case StandStage::WavePush:
        case StandStage::WaveRecoveryLift:
        case StandStage::WaveRecoverySwing:
        case StandStage::WaveRecoveryLower:
            switch (gaitState.walkMode) {
                case WalkMode::SingleWave:
                    return "wave walk";
                case WalkMode::DualRipple:
                    return "dual ripple walk";
                case WalkMode::ContinuousRippleV4DistributedStance:
                    return "continuous ripple v4";
                case WalkMode::ContinuousRippleV6FullIk:
                    return "continuous ripple v6";
                case WalkMode::ContinuousRippleV6ObstacleAvoidance:
                    return "obstacle avoidance";
                case WalkMode::TurnClockwise:
                case WalkMode::TurnCounterClockwise:
                case WalkMode::TurnClockwiseFast:
                case WalkMode::TurnCounterClockwiseFast:
                    return "turning";
                case WalkMode::CalculatedPath:
                    return "calculated path";
                default:
                    return "walking";
            }
        case StandStage::WaveStopped:
            return "movement stopped";
        case StandStage::OutputsOff:
            return "leg outputs off";
        case StandStage::Fault:
            return "fault";
    }
    return "unknown";
}

void setStandFault(const char *message) {
    snprintf(standState.fault, sizeof(standState.fault), "%s", message);
    standState.stage = StandStage::Fault;
    Serial.printf("Stand fault: %s\n", standState.fault);
}

bool writeLegJointPrecise(uint8_t legIndex, uint8_t jointIndex, float angleDeg) {
    const RobotConfig::ServoMapping &mapping = RobotConfig::kLegServos[legIndex].joints[jointIndex];
    const bool solderedBoard = mapping.board == RobotConfig::PwmBoard::Soldered;
    if ((solderedBoard && !pcaState.solderedReady) || (!solderedBoard && !pcaState.ready)) {
        setStandFault(solderedBoard ? "PCA9685 0x41 disconnected" : "PCA9685 0x40 disconnected");
        return false;
    }

    const float safeAngleDeg = fminf(180.0F, fmaxf(0.0F, angleDeg));
    const int roundedAngleDeg = static_cast<int>(lroundf(safeAngleDeg));

    const bool exactInteger = fabsf( safeAngleDeg - static_cast<float>(roundedAngleDeg)) < 0.0001F;
    const int pulse = exactInteger ? RobotConfig::servoAngleToPulse(roundedAngleDeg)
            : static_cast<int>(lroundf( static_cast<float>(RobotConfig::kServoPulseMin) + safeAngleDeg / 180.0F * static_cast<float>(RobotConfig::kServoPulseMax - RobotConfig::kServoPulseMin)));

    if (solderedBoard) {
        solderedPwm.setPWM(mapping.channel, 0, pulse);
    } else {
        aimPwm.setPWM(mapping.channel, 0, pulse);
    }

    standState.legCommandDegrees[legIndex][jointIndex] = roundedAngleDeg;
    standState.legCommandKnown[legIndex][jointIndex] = true;
    return true;
}

bool allLegCommandsKnown() {
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
            if (!standState.legCommandKnown[legIndex][jointIndex]) {
                return false;
            }
        }
    }

    return true;
}

bool writeLegPose(uint8_t legIndex, const float pose[RobotConfig::kJointCount]) {
    for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
        if (!writeLegJointPrecise(legIndex, jointIndex, pose[jointIndex])) {
            return false;
        }
    }

    return true;
}

bool writeActiveUniformStandPose(const float pose[RobotConfig::kJointCount]) {
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        if (!writeLegPose(legIndex, pose)) {
            return false;
        }
    }

    return true;
}

void disableAllLegOutputs() {
    for (uint8_t legIndex = 0; legIndex < RobotConfig::kLegCount; ++legIndex) {
        for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
            standState.legCommandKnown[legIndex][jointIndex] = false;
            const RobotConfig::ServoMapping &mapping = RobotConfig::kLegServos[legIndex].joints[jointIndex];

            if (mapping.board == RobotConfig::PwmBoard::Soldered) {
                if (pcaState.solderedReady) {
                    solderedPwm.setPWM(mapping.channel, 0, 4096);
                }
            } else if (pcaState.ready) {
                aimPwm.setPWM(mapping.channel, 0, 4096);
            }
        }
    }
}

void startStandupSequence() {
    standState.mode = StandMode::FullRobot;
    standState.fault[0] = '\0';
    standState.progress = 0.0F;
    float startupPose[RobotConfig::kJointCount] = {
        static_cast<float>(kOriginalStandStartupPose[0]),
        static_cast<float>(kOriginalStandStartupPose[1]),
        static_cast<float>(kOriginalStandStartupPose[2]),
    };

    if (!writeActiveUniformStandPose(startupPose)) {
        return;
    }

    standState.stage = StandStage::StartupPoseHold;
    standState.stageStartedAtMs = millis();
    standState.lastServoUpdateAtMs = 0;
    Serial.printf("Pose 90/180/170");
}

bool startWalkingPoseSequence() {
    standState.mode = StandMode::FullRobot;
    standState.fault[0] = '\0';
    standState.progress = 0.0F;

    float all90Pose[RobotConfig::kJointCount] = {
        static_cast<float>(kAll90StandingPose[0]),
        static_cast<float>(kAll90StandingPose[1]),
        static_cast<float>(kAll90StandingPose[2]),
    };

    if (!writeActiveUniformStandPose(all90Pose)) {
        return false;
    }

    standState.stage = StandStage::MovingToWalkingPose;
    standState.stageStartedAtMs = millis();
    standState.lastServoUpdateAtMs = 0;
    Serial.println("Pose 90/150/140");
    return true;
}

void updateLegOperation() {
    if (!legOperationIsActive()) {
        return;
    }

    // Walking / Turning
    const uint32_t nowMs = millis();
    if (updateWaveSequence(nowMs)) {
        return;
    }

    // Stand up pose -> 90/90/90
    if (standState.stage == StandStage::StartupPoseHold) {
        standState.progress = fminf( 1.0F, static_cast<float>(nowMs - standState.stageStartedAtMs) / static_cast<float>(kStandStartupPoseHoldMs));
        if (standState.progress < 1.0F) {
            return;
        }

        standState.stage = StandStage::MovingAllTo90;
        standState.stageStartedAtMs = nowMs;
        standState.lastServoUpdateAtMs = 0;
        standState.progress = 0.0F;
        Serial.println("Pose 90/90/90");
        return;
    }

    // 90/90/90 -> Walking pose
    if (standState.stage == StandStage::All90Pause) {
        standState.progress = fminf( 1.0F, static_cast<float>(nowMs - standState.stageStartedAtMs) / static_cast<float>(kAll90ToWalkingPauseMs));
        if (standState.progress < 1.0F) {
            return;
        }
        Serial.println("Pose 90/150/140");
        if (!startWalkingPoseSequence()) {
            return;
        }
        return;
    }

    // Move servers form 90/90/90 to 90/150/140
    if (standState.stage == StandStage::MovingToWalkingPose) {
        if (nowMs - standState.lastServoUpdateAtMs < kStandServoUpdateIntervalMs) {
            return;
        }
        standState.lastServoUpdateAtMs = nowMs;

        standState.progress = fminf( 1.0F, static_cast<float>(nowMs - standState.stageStartedAtMs) / static_cast<float>(kWalkingPoseTransitionMs));

        float pose[RobotConfig::kJointCount] = {};
        for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
            const float start = kAll90StandingPose[jointIndex];
            const float target = kWalkingTestPose[jointIndex];
            pose[jointIndex] = start + (target - start) * standState.progress;
        }

        if (!writeActiveUniformStandPose(pose)) {
            return;
        }

        if (standState.progress >= 1.0F) {
            standState.stage = StandStage::WalkingPoseComplete;
            Serial.println("Pose ready");
        }
        return;
    }

    if (nowMs - standState.lastServoUpdateAtMs < kStandServoUpdateIntervalMs) {
        return;
    }

    standState.lastServoUpdateAtMs = nowMs;

    // Start -> 90/90/90
    standState.progress = fminf( 1.0F, static_cast<float>(nowMs - standState.stageStartedAtMs) / static_cast<float>(kStandAllTo90TransitionMs));
    float pose[RobotConfig::kJointCount] = {};
    for (uint8_t jointIndex = 0; jointIndex < RobotConfig::kJointCount; ++jointIndex) {
        const float start = kOriginalStandStartupPose[jointIndex];
        const float target = kAll90StandingPose[jointIndex];
        pose[jointIndex] = start + (target - start) * standState.progress;
    }

    if (!writeActiveUniformStandPose(pose)) {
        return;
    }

    if (standState.progress >= 1.0F) {
        standState.stage = StandStage::All90Pause;
        standState.stageStartedAtMs = nowMs;
        standState.progress = 0.0F;
        Serial.println("Pose 90/90/90 ready");
    }
}

} // namespace Hexapod
