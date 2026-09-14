#include <Arduino.h>

#include "HexapodState.h"
#include "CannonPreparation.h"
#include "GaitController.h"
#include "ObstacleNavigation.h"
#include "CannonController.h"
#include "ToFSensors.h"
#include "WebStatus.h"

namespace Hexapod {

void sendStatus() {
    const uint32_t nowMs = millis();
    const uint32_t ageMs = aimState.hasMessage ? nowMs - aimState.lastMessageAtMs : 0U;
    const bool dataRecent = aimState.hasMessage && ageMs <= kFreshMessageTimeoutMs;
    const bool controlFresh = aimTrackingDataIsFresh(nowMs);
    const int32_t remainingChargeTime = chargeServoIsActive() ? static_cast<int32_t>(cannonState.chargeStopAtMs - nowMs) : 0;
    const uint32_t chargeRemainingMs = remainingChargeTime > 0 ? static_cast<uint32_t>(remainingChargeTime) : 0U;

    const bool centeredWalkPose = standState.stage == StandStage::WalkingPoseComplete && allLegCommandsMatchWalkingPose();
    const bool stoppedWavePose = standState.stage == StandStage::WaveStopped && gaitState.walkMode == WalkMode::SingleWave && allLegCommandsMatchWaveRearPose();
    const bool stoppedDualPose = standState.stage == StandStage::WaveStopped && gaitState.walkMode == WalkMode::DualRipple && allLegCommandsKnown();
    const bool stoppedRipplePose = standState.stage == StandStage::WaveStopped && allLegCommandsMatchContinuousRippleStopPose();
    const bool fullRobot = standState.mode == StandMode::FullRobot;

    String json;
    json.reserve(2100);
    json += "{\"hasData\":";
    json += aimState.hasMessage ? "true" : "false";
    json += ",\"dataRecent\":";
    json += dataRecent ? "true" : "false";
    json += ",\"controlFresh\":";
    json += controlFresh ? "true" : "false";
    json += ",\"aimTagsVisible\":";
    json += aimState.tagsVisible ? "true" : "false";
    json += ",\"cannonTagDetected\":";
    json += aimState.cannonTagDetected ? "true" : "false";
    json += ",\"targetTagDetected\":";
    json += aimState.targetTagDetected ? "true" : "false";
    json += ",\"direction\":\"";
    json += aimDirectionName(aimState.latestErrorDegrees);
    json += "\",\"errorDeg\":";
    json += String(aimState.latestErrorDegrees, 2);
    json += ",\"cameraTargetDistanceCm\":";
    if (aimState.hasTargetDistance) {
        json += String(aimState.targetDistanceCm, 1);
    } else {
        json += "null";
    }

    json += ",\"pcaReady\":";
    json += pcaState.ready ? "true" : "false";
    json += ",\"pca41Ready\":";
    json += pcaState.solderedReady ? "true" : "false";
    json += ",\"waveCanStart\":";
    json += fullRobot && (centeredWalkPose || stoppedWavePose) ? "true" : "false";
    json += ",\"dualCanStart\":";
    json += fullRobot && (centeredWalkPose || stoppedWavePose || stoppedDualPose) ? "true" : "false";
    json += ",\"continuousRippleCanStart\":";
    json += fullRobot && (centeredWalkPose || stoppedRipplePose) ? "true" : "false";
    json += ",\"turnCanStart\":";
    json += fullRobot && centeredWalkPose ? "true" : "false";
    json += ",\"waveCanStop\":";
    json += ((waveStageIsActive() && !gaitState.waveStopRequested) || obstacleNavigationActive()) ? "true" : "false";

    json += ",\"obstacleNavigationCanStart\":";
    json += obstacleNavigationCanStart(nowMs) ? "true" : "false";
    json += ",\"obstacleNavigationActive\":";
    json += obstacleNavigationActive() ? "true" : "false";
    json += ",\"obstacleNavigationState\":\"";
    json += obstacleNavigationStateName();
    json += "\",\"obstacleNavigationReason\":\"";
    json += obstacleNavigationReason();
    json += '"';

    json += ",\"servoEnabled\":";
    json += cannonState.aimTrackingEnabled ? "true" : "false";
    json += ",\"manualAimActive\":";
    json += cannonState.manualAimActive ? "true" : "false";
    json += ",\"shootActive\":";
    json += shootServoIsActive() ? "true" : "false";
    json += ",\"tiltActive\":";
    json += tiltServoIsActive() ? "true" : "false";
    json += ",\"chargeActive\":";
    json += chargeServoIsActive() ? "true" : "false";
    json += ",\"chargeAction\":\"";
    json += chargeActionName(cannonState.chargeAction);
    json += "\",\"chargeRemainingMs\":";
    json += chargeRemainingMs;
    json += ",\"estimatedChargeWoundDurationMs\":";
    json += cannonState.estimatedWoundDurationMs;
    json += ",\"chargeAtEstimatedHome\":";
    json += cannonState.estimatedWoundDurationMs == 0U ? "true" : "false";

    json += ",\"cannonPreparationCanStart\":";
    json += cannonPreparationCanStart() ? "true" : "false";
    json += ",\"cannonPreparationActive\":";
    json += cannonPreparationActive() ? "true" : "false";
    json += ",\"cannonPreparationReady\":";
    json += cannonPreparationReady() ? "true" : "false";
    json += ",\"cannonPreparationAwaitingConfirmation\":";
    json += cannonPreparationAwaitingConfirmation() ? "true" : "false";
    json += ",\"cannonPreparationAwaitingTiltHomeConfirmation\":";
    json += cannonPreparationAwaitingTiltHomeConfirmation() ? "true" : "false";
    json += ",\"cannonPreparationAwaitingTiltUpConfirmation\":";
    json += cannonPreparationAwaitingTiltUpConfirmation() ? "true" : "false";
    json += ",\"cannonPreparationAwaitingUnwindConfirmation\":";
    json += cannonPreparationAwaitingUnwindConfirmation() ? "true" : "false";
    json += ",\"cannonPreparationCanAddChargeStep\":";
    json += cannonPreparationCanAddChargeStep() ? "true" : "false";
    json += ",\"cannonPreparationState\":\"";
    json += cannonPreparationStateName();
    json += "\",\"cannonPreparationReason\":\"";
    json += cannonPreparationReason();
    json += "\",\"cannonPreparationDistanceCm\":";
    if (cannonPreparationDistanceCm() > 0.0F) {
        json += String(cannonPreparationDistanceCm(), 1);
    } else {
        json += "null";
    }
    json += ",\"cannonPreparationTiltUpMs\":";
    json += static_cast<unsigned int>(cannonPreparationTiltUpMs());
    json += ",\"cannonPreparationPowerLevel\":";
    json += static_cast<unsigned int>(cannonPreparationPowerLevel());
    json += ",\"cannonPreparationBaseChargeDurationMs\":";
    json += cannonPreparationBaseChargeDurationMs();
    json += ",\"cannonPreparationPlannedChargeDurationMs\":";
    json += cannonPreparationPlannedChargeDurationMs();
    json += ",\"cannonPreparationChargeCorrectionMs\":";
    json += cannonPreparationChargeCorrectionMs();

    json += ",\"tof\":{\"started\":";
    json += tofSensorsStarted() ? "true" : "false";
    json += ",\"fault\":\"";
    json += tofSensorFault();
    json += "\",\"sensors\":[";
    for (uint8_t index = 0; index < RobotConfig::kToFSensorCount; ++index) {
        if (index > 0U) {
            json += ',';
        }
        const RobotConfig::ToFSensorConfig &config = RobotConfig::kToFSensors[index];
        const DistanceReading &reading = tofSensorReading(index);
        const bool hasTimestamp = reading.timestampMs != 0U;
        const uint32_t sensorAgeMs = hasTimestamp ? nowMs - reading.timestampMs : 0U;
        const bool fresh = tofSensorInitialized(index) && reading.valid && hasTimestamp && sensorAgeMs <= RobotConfig::kSensorStaleTimeoutMs;
        json += "{\"channel\":";
        json += static_cast<unsigned int>(config.muxChannel);
        json += ",\"initialized\":";
        json += tofSensorInitialized(index) ? "true" : "false";
        json += ",\"valid\":";
        json += fresh ? "true" : "false";
        json += ",\"rawMm\":";
        json += reading.rawMm;
        json += ",\"filteredMm\":";
        if (fresh) {
            json += reading.filteredMm;
        } else {
            json += "null";
        }
        json += ",\"ageMs\":";
        if (hasTimestamp) {
            json += sensorAgeMs;
        } else {
            json += "null";
        }
        json += ",\"timedOut\":";
        json += reading.timedOut ? "true" : "false";
        json += '}';
    }
    json += "]}";

    json += ",\"standActive\":";
    json += legOperationIsActive() ? "true" : "false";
    json += ",\"standMode\":\"";
    json += standModeName();
    json += "\",\"standStage\":\"";
    json += standStageName();
    json += "\",\"standProgressPct\":";
    json += String(standState.progress * 100.0F, 1);
    json += ",\"standFault\":\"";
    json += standState.fault;
    json += "\"}";

    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", json);
}

void sendAimTrackingControlStatus() {
    const uint32_t nowMs = millis();
    String json;
    json.reserve(240);
    json += "{\"receiverTimeMs\":";
    json += nowMs;
    json += ",\"servoEnabled\":";
    json += cannonState.aimTrackingEnabled ? "true" : "false";
    json += ",\"hasData\":";
    json += aimState.hasMessage ? "true" : "false";
    json += ",\"fresh\":";
    json += aimTrackingDataIsFresh(nowMs) ? "true" : "false";
    json += ",\"aimTagsVisible\":";
    json += aimState.tagsVisible ? "true" : "false";
    json += ",\"sequence\":";
    if (aimState.hasMessage) {
        json += aimState.latestSequence;
    } else {
        json += "null";
    }
    json += ",\"acceptedAimMessages\":";
    json += aimState.acceptedMessageCount;
    json += ",\"pcaReady\":";
    json += pcaState.ready ? "true" : "false";
    json += ",\"servoAction\":\"";
    json += aimServoActionName(cannonState.appliedAimAction);
    json += "\",\"servoCommand\":";
    if (cannonState.appliedAimCommand < 0) {
        json += "null";
    } else {
        json += cannonState.appliedAimCommand;
    }
    json += "}";
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", json);
}

} // namespace Hexapod
