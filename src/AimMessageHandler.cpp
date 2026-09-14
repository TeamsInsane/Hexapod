#include <Arduino.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "AimMessageHandler.h"
#include "CannonController.h"

namespace Hexapod {

bool readFloatArgument(const char *name, float &value) {
    if (!webServer.hasArg(name)) {
        return false;
    }

    const String text = webServer.arg(name);
    char *end = nullptr;
    errno = 0;
    const float parsed = strtof(text.c_str(), &end);
    if (errno == ERANGE || end == text.c_str() || *end != '\0' || !isfinite(parsed)) {
        return false;
    }

    value = parsed;
    return true;
}

const ShotCalibrationPoint &findClosestShotCalibration(float distanceCm) {
    size_t bestIndex = 0;
    float bestDifference = fabsf(distanceCm - kShotCalibration[bestIndex].representativeRangeCm);

    for (size_t index = 1; index < sizeof(kShotCalibration) / sizeof(kShotCalibration[0]); ++index) {
        const ShotCalibrationPoint &candidate = kShotCalibration[index];
        const ShotCalibrationPoint &best = kShotCalibration[bestIndex];

        const float difference = fabsf(distanceCm - candidate.representativeRangeCm);
        const bool sameDifference = fabsf(difference - bestDifference) <= 0.001F;

        const float candidateSpread = candidate.observedMaximumCm - candidate.observedMinimumCm;
        const float bestSpread = best.observedMaximumCm - best.observedMinimumCm;
        const bool betterTieBreak = candidateSpread < bestSpread - 0.001F || (fabsf(candidateSpread - bestSpread) <= 0.001F && candidate.tiltUpMs < best.tiltUpMs);

        if (difference < bestDifference - 0.001F || (sameDifference && betterTieBreak)) {
            bestIndex = index;
            bestDifference = difference;
        }
    }

    return kShotCalibration[bestIndex];
}

bool readSequenceArgument(uint32_t &value) {
    if (!webServer.hasArg("sequence")) {
        return false;
    }

    const String text = webServer.arg("sequence");
    if (text.isEmpty() || text[0] == '-') {
        return false;
    }

    char *end = nullptr;
    errno = 0;
    const unsigned long long parsed = strtoull(text.c_str(), &end, 10);
    if (errno == ERANGE || end == text.c_str() || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }

    value = static_cast<uint32_t>(parsed);
    return true;
}

bool readBooleanArgument(const char *name, bool &value) {
    if (!webServer.hasArg(name)) {
        return false;
    }

    const String text = webServer.arg(name);
    if (text == "1" || text == "true") {
        value = true;
        return true;
    }

    if (text == "0" || text == "false") {
        value = false;
        return true;
    }
    return false;
}

// Receives error_deg, cannon_heading_deg, target_heading_deg, sequence, ?target_distance_cm
void handleAimMessage() {
    float errorDegrees = 0.0F;
    float cannonHeadingDegrees = 0.0F;
    float targetHeadingDegrees = 0.0F;
    float targetDistanceCm = 0.0F;
    uint32_t sequence = 0;

    if (!readFloatArgument("error_deg", errorDegrees) || !readFloatArgument("cannon_heading_deg", cannonHeadingDegrees) || !readFloatArgument("target_heading_deg", targetHeadingDegrees) || !readSequenceArgument(sequence)) {
        webServer.send( 400, "text/plain", "Required form fields: error_deg, cannon_heading_deg, target_heading_deg, sequence" );
        return;
    }

    const bool distanceProvided = webServer.hasArg("target_distance_cm");
    if (distanceProvided && !readFloatArgument("target_distance_cm", targetDistanceCm)) {
        webServer.send(400, "text/plain", "target_distance_cm must be a finite number when provided");
        return;
    }

    if (errorDegrees < -180.0F || errorDegrees > 180.0F || cannonHeadingDegrees < 0.0F || cannonHeadingDegrees >= 360.0F || targetHeadingDegrees < 0.0F || targetHeadingDegrees >= 360.0F
            || (distanceProvided && (targetDistanceCm <= 0.0F || targetDistanceCm > 1000.0F))) {
        webServer.send(422, "text/plain", "Aim values are outside valid ranges");
        return;
    }

    aimState.latestErrorDegrees = errorDegrees;
    aimState.latestSequence = sequence;
    aimState.latestCameraFrameSequence = sequence;
    ++aimState.acceptedMessageCount;
    aimState.lastMessageAtMs = millis();

    if (distanceProvided) {
        aimState.hasTargetDistance = true;
        aimState.targetDistanceCm = targetDistanceCm;
        aimState.targetDistanceAtMs = aimState.lastMessageAtMs;
    } else {
        aimState.hasTargetDistance = false;
        aimState.targetDistanceAtMs = 0U;
    }

    aimState.tagsVisible = true;
    aimState.cannonTagDetected = true;
    aimState.targetTagDetected = true;
    aimState.hasMessage = true;

    char cameraDistanceText[16] = "--";
    if (aimState.hasTargetDistance) {
        snprintf(cameraDistanceText, sizeof(cameraDistanceText), "%.1f cm", aimState.targetDistanceCm);
    }

    char logLine[kMessageLogLineLength] = "";
    snprintf(logLine, sizeof(logLine), "AIM #%lu | %-7s | error=%7.2f deg | cannon=%7.2f deg | target=%7.2f deg | camera range=%s | from %s",
             static_cast<unsigned long>(aimState.latestSequence), aimDirectionName(aimState.latestErrorDegrees), aimState.latestErrorDegrees, cannonHeadingDegrees, targetHeadingDegrees, cameraDistanceText,
             webServer.client().remoteIP().toString().c_str());
    Serial.println(logLine);
    updateAimServo();

    const uint32_t responseTimeMs = millis();
    String response;
    response.reserve(280);
    response += "{\"received\":true,\"sequence\":";
    response += aimState.latestSequence;
    response += ",\"acceptedAimMessages\":";
    response += aimState.acceptedMessageCount;
    response += ",\"receivedAtMs\":";
    response += aimState.lastMessageAtMs;
    response += ",\"responseAtMs\":";
    response += responseTimeMs;
    response += ",\"direction\":\"";
    response += aimDirectionName(aimState.latestErrorDegrees);
    response += "\",\"errorDeg\":";
    response += String(aimState.latestErrorDegrees, 2);
    response += ",\"cameraTargetDistanceCm\":";
    if (aimState.hasTargetDistance) {
        response += String(aimState.targetDistanceCm, 1);
    } else {
        response += "null";
    }
    response += ",\"servoEnabled\":";
    response += cannonState.aimTrackingEnabled ? "true" : "false";
    response += ",\"servoAction\":\"";
    response += aimServoActionName(cannonState.appliedAimAction);
    response += "\",\"servoCommand\":";

    if (cannonState.appliedAimCommand < 0) {
        response += "null";
    } else {
        response += cannonState.appliedAimCommand;
    }

    response += ",\"pcaReady\":";
    response += pcaState.ready ? "true" : "false";
    response += "}";

    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", response);
}

void handleAimLost() {
    uint32_t frameSequence = aimState.latestCameraFrameSequence + 1U;
    if (webServer.hasArg("sequence") && !readSequenceArgument(frameSequence)) {
        webServer.send(400, "text/plain", "Optional sequence must be an unsigned 32-bit integer");
        return;
    }

    bool cannonDetected = false;
    bool targetDetected = false;
    if (webServer.hasArg("cannon_detected") && !readBooleanArgument("cannon_detected", cannonDetected)) {
        webServer.send(400, "text/plain", "cannon_detected must be true/false");
        return;
    }

    if (webServer.hasArg("target_detected") && !readBooleanArgument("target_detected", targetDetected)) {
        webServer.send(400, "text/plain", "target_detected must be true/false");
        return;
    }

    if (cannonDetected && targetDetected) {
        webServer.send(422, "text/plain", "api/aim");
        return;
    }

    aimState.latestCameraFrameSequence = frameSequence;
    aimState.cannonTagDetected = cannonDetected;
    aimState.targetTagDetected = targetDetected;
    aimState.tagsVisible = false;
    aimState.hasTargetDistance = false;
    aimState.targetDistanceAtMs = 0U;

    updateAimServo();

    String response;
    response.reserve(180);
    response += "{\"received\":true,\"frameSequence\":";
    response += frameSequence;
    response += ",\"aimTagsVisible\":false,\"cannonTagDetected\":";
    response += cannonDetected ? "true" : "false";
    response += ",\"targetTagDetected\":";
    response += targetDetected ? "true" : "false";
    response += ",\"servoAction\":\"";
    response += aimServoActionName(cannonState.appliedAimAction);
    response += "\",\"receiverTimeMs\":";
    response += millis();
    response += "}";
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send(200, "application/json", response);
}

} // namespace Hexapod
