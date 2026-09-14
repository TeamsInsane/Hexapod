#include "ObstacleNavigation.h"

#include <stdarg.h>
#include <stdio.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "ToFSensors.h"

namespace Hexapod {
namespace {

enum class ObstacleState : uint8_t {
    Idle,
    Forward,
    StoppingForObstacle,
    TurningLeft,
    TurningRight,
    StoppingAfterTurn,
    Blocked,
    SensorFault,
    UserStopping,
};

ObstacleState state = ObstacleState::Idle;
char reason[160] = "inactive";
uint8_t obstacleSamples = 0;
uint8_t clearSamples = 0;
uint32_t lastFrontTimestampMs = 0;
bool preferLeftOnTie = true;

void setReason(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(reason, sizeof(reason), format, args);
    va_end(args);
}

uint8_t sensorIndexForRole(ToFSensorRole role) {
    return static_cast<uint8_t>(role);
}

bool communicationFresh(ToFSensorRole role, uint32_t nowMs) {
    const uint8_t index = sensorIndexForRole(role);
    const DistanceReading &reading = tofSensorReading(index);
    return tofSensorInitialized(index) && reading.timestampMs != 0U && nowMs - reading.timestampMs <= RobotConfig::kSensorStaleTimeoutMs && !reading.timedOut;
}

bool allCommunicationFresh(uint32_t nowMs) {
    return communicationFresh(ToFSensorRole::Front, nowMs) && communicationFresh(ToFSensorRole::Left, nowMs) && communicationFresh(ToFSensorRole::Right, nowMs);
}

const DistanceReading &roleReading(ToFSensorRole role) {
    return tofSensorReading(sensorIndexForRole(role));
}

uint16_t effectiveClearanceMm(ToFSensorRole role) {
    const DistanceReading &reading = roleReading(role);
    if (reading.valid) {
        return reading.filteredMm;
    }

    return reading.rawMm >= 8190U ? UINT16_MAX : 0U;
}

void resetForwardConfirmation(uint32_t nowMs) {
    obstacleSamples = 0;
    clearSamples = 0;
    lastFrontTimestampMs = communicationFresh(ToFSensorRole::Front, nowMs) ? roleReading(ToFSensorRole::Front).timestampMs : 0U;
}

void beginFreshV6(uint32_t nowMs, bool includeCenterPair) {
    gaitState.walkMode = WalkMode::ContinuousRippleV6ObstacleAvoidance;
    gaitState.ripplePriming = true;
    gaitState.rippleFullPriming = includeCenterPair;
    gaitState.ripplePrimeStep = 0U;
    gaitState.waveStopRequested = false;
    gaitState.completedTurnCycles = 0U;
    resetActiveWalkGroup();
    beginWaveStage(StandStage::WaveLift, nowMs);
    state = ObstacleState::Forward;
    resetForwardConfirmation(nowMs);
    setReason("walking forward");
}

void resumeStoppedV6(uint32_t nowMs) {
    gaitState.waveStopRequested = false;
    beginWaveStage(StandStage::WaveLift, nowMs);
    state = ObstacleState::Forward;

    resetForwardConfirmation(nowMs);
    setReason("Path clear");
}

void requestGroundedStop(ObstacleState stoppingState, const char *message) {
    gaitState.waveStopRequested = true;
    state = stoppingState;
    setReason("%s", message);
    Serial.printf("Obstacle navigation: %s\n", message);
}

void startTurn(bool left, uint32_t nowMs) {
    gaitState.ripplePriming = false;
    gaitState.ripplePrimeStep = 0U;
    gaitState.waveStopRequested = false;
    gaitState.completedTurnCycles = 0U;

    resetActiveWalkGroup();
    gaitState.walkMode = left ? WalkMode::TurnCounterClockwise : WalkMode::TurnClockwise;
    beginWaveStage(StandStage::WaveLift, nowMs);
    state = left ? ObstacleState::TurningLeft : ObstacleState::TurningRight;
    clearSamples = 0U;
    lastFrontTimestampMs = roleReading(ToFSensorRole::Front).timestampMs;
    setReason("turning %s", left ? "left" : "right");
}

void assessPathAndContinue(uint32_t nowMs) {
    if (!allCommunicationFresh(nowMs)) {
        state = ObstacleState::SensorFault;
        setReason("Sensor data stale");
        return;
    }

    const uint16_t frontMm = effectiveClearanceMm(ToFSensorRole::Front);
    if (frontMm > RobotConfig::kFrontClearMm) {
        if (gaitState.walkMode == WalkMode::ContinuousRippleV6ObstacleAvoidance && standState.stage == StandStage::WaveStopped) {
            resumeStoppedV6(nowMs);
        } else {
            beginFreshV6(nowMs, turnModeActive());
        }

        return;
    }

    const uint16_t leftMm = effectiveClearanceMm(ToFSensorRole::Left);
    const uint16_t rightMm = effectiveClearanceMm(ToFSensorRole::Right);
    const bool leftBlocked = leftMm < RobotConfig::kSideBlockedMm;
    const bool rightBlocked = rightMm < RobotConfig::kSideBlockedMm;

    bool turnLeft = false;
    if (leftBlocked && rightBlocked) {
        if (leftMm != rightMm) {
            turnLeft = leftMm > rightMm;
        } else {
            turnLeft = preferLeftOnTie;
            preferLeftOnTie = !preferLeftOnTie;
        }
    } else if (rightBlocked) {
        turnLeft = true;
    } else if (leftBlocked) {
        turnLeft = false;
    } else if (leftMm > rightMm + RobotConfig::kSideDirectionMarginMm) {
        turnLeft = true;
    } else if (rightMm > leftMm + RobotConfig::kSideDirectionMarginMm) {
        turnLeft = false;
    } else {
        turnLeft = preferLeftOnTie;
        preferLeftOnTie = !preferLeftOnTie;
    }

    startTurn(turnLeft, nowMs);
}

void updateForward(uint32_t nowMs) {
    if (!allCommunicationFresh(nowMs)) {
        requestGroundedStop( ObstacleState::SensorFault, "Sensor data stale");
        return;
    }

    const DistanceReading &front = roleReading(ToFSensorRole::Front);
    if (front.timestampMs == lastFrontTimestampMs) {
        return;
    }
    lastFrontTimestampMs = front.timestampMs;
    const uint16_t frontMm = effectiveClearanceMm(ToFSensorRole::Front);
    if (front.valid && frontMm <= RobotConfig::kFrontHardStopMm) {
        obstacleSamples = RobotConfig::kObstacleConfirmSamples;
    } else if (front.valid && frontMm < RobotConfig::kFrontTurnTriggerMm) {
        if (obstacleSamples < RobotConfig::kObstacleConfirmSamples) {
            ++obstacleSamples;
        }
    } else {
        obstacleSamples = 0U;
    }

    if (obstacleSamples >= RobotConfig::kObstacleConfirmSamples) {
        char message[96] = "";
        snprintf(message, sizeof(message), "Front obstacle: %u mm", static_cast<unsigned int>(frontMm));
        requestGroundedStop(ObstacleState::StoppingForObstacle, message);
    }
}

void updateTurning(uint32_t nowMs) {
    if (!allCommunicationFresh(nowMs)) {
        requestGroundedStop( ObstacleState::SensorFault, "Sensor data stale");
        return;
    }

    if (standState.stage != StandStage::WavePush && gaitState.completedTurnCycles == 0U) {
        return;
    }

    const DistanceReading &front = roleReading(ToFSensorRole::Front);
    if (front.timestampMs == lastFrontTimestampMs) {
        return;
    }

    lastFrontTimestampMs = front.timestampMs;
    const uint16_t frontMm = effectiveClearanceMm(ToFSensorRole::Front);
    if (frontMm > RobotConfig::kFrontClearMm) {
        if (clearSamples < RobotConfig::kClearConfirmSamples) {
            ++clearSamples;
        }
    } else {
        clearSamples = 0U;
    }

    if (clearSamples >= RobotConfig::kClearConfirmSamples) {
        requestGroundedStop( ObstacleState::StoppingAfterTurn, "front path clear");
    }
}

} // namespace

bool obstacleNavigationActive() {
    return state != ObstacleState::Idle;
}

bool obstacleNavigationCanStart(uint32_t nowMs) {
    return state == ObstacleState::Idle && allCommunicationFresh(nowMs);
}

const char *obstacleNavigationStateName() {
    switch (state) {
        case ObstacleState::Idle: return "idle";
        case ObstacleState::Forward: return "walking forward";
        case ObstacleState::StoppingForObstacle: return "stopping for obstacle";
        case ObstacleState::TurningLeft: return "turning left";
        case ObstacleState::TurningRight: return "turning right";
        case ObstacleState::StoppingAfterTurn: return "finishing turn";
        case ObstacleState::Blocked: return "blocked";
        case ObstacleState::SensorFault: return "sensor fault";
        case ObstacleState::UserStopping: return "user stop pending";
    }

    return "idle";
}

const char *obstacleNavigationReason() {
    return reason;
}

void startObstacleNavigation(uint32_t nowMs) {
    state = ObstacleState::Forward;
    resetForwardConfirmation(nowMs);
    setReason("Walking; trigger %u mm", static_cast<unsigned int>(RobotConfig::kFrontTurnTriggerMm));
}

void updateObstacleNavigation(uint32_t nowMs) {
    if (state == ObstacleState::Idle) {
        return;
    }

    if (standState.stage == StandStage::Fault) {
        state = ObstacleState::SensorFault;
        setReason("gait fault: %s", standState.fault);
        return;
    }

    switch (state) {
        case ObstacleState::Forward:
            updateForward(nowMs);
            break;

        case ObstacleState::StoppingForObstacle:
            if (standState.stage == StandStage::WaveStopped) {
                assessPathAndContinue(nowMs);
            }
            break;

        case ObstacleState::TurningLeft:
        case ObstacleState::TurningRight:
            if (standState.stage == StandStage::WaveStopped) {
                state = ObstacleState::Blocked;
                setReason("Blocked after %u turns", static_cast<unsigned int>(gaitState.completedTurnCycles));
            } else {
                updateTurning(nowMs);
            }
            break;

        case ObstacleState::StoppingAfterTurn:
            if (standState.stage == StandStage::WaveStopped) {
                assessPathAndContinue(nowMs);
            }
            break;

        case ObstacleState::SensorFault:
            if (waveStageIsActive()) {
                gaitState.waveStopRequested = true;
            } else if (allCommunicationFresh(nowMs) && standState.stage == StandStage::WaveStopped) {
                setReason("Sensors restored");
                assessPathAndContinue(nowMs);
            }
            break;

        case ObstacleState::UserStopping:
            if (standState.stage == StandStage::WaveStopped) {
                state = ObstacleState::Idle;
                setReason("Stopped");
            }
            break;

        case ObstacleState::Blocked:
        case ObstacleState::Idle:
            break;
    }
}

void requestObstacleNavigationStop() {
    if (state == ObstacleState::Idle) {
        return;
    }

    if (state == ObstacleState::Blocked || state == ObstacleState::SensorFault) {
        state = ObstacleState::Idle;
        setReason("Stopped");
        return;
    }

    gaitState.waveStopRequested = true;
    state = ObstacleState::UserStopping;
    setReason("Stopping");
}

void cancelObstacleNavigation() {
    state = ObstacleState::Idle;
    setReason("Cancelled");
}

} // namespace Hexapod
