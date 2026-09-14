#include <Arduino.h>
#include <stdio.h>

#include "HexapodState.h"
#include "GaitController.h"
#include "NavigationController.h"
#include "ObstacleNavigation.h"
#include "CannonController.h"
#include "WebMovement.h"


namespace Hexapod {

void startStandupRequest() {
    if (legOperationIsActive()) {
        webServer.send(409, "text/plain", "Stand-up active.");
        return;
    }

    refreshPca9685Boards(true);
    if (!pcaState.ready || !pcaState.solderedReady) {
        webServer.send(503, "text/plain", "Both PCA9685 boards 0x40 and 0x41 must be connected");
        return;
    }

    cancelManualAim();
    cannonState.aimTrackingEnabled = false;
    aimPwm.setPWM(kAimServoChannel, 0, 4096);
    cannonState.appliedAimAction = AimServoAction::Off;
    cannonState.appliedAimCommand = -1;
    stopShootServo();
    stopTiltServo();
    stopChargeServo();
    disableAllLegOutputs();

    startStandupSequence();

    if (standState.stage == StandStage::Fault) {
        webServer.send(500, "text/plain", standState.fault);
        return;
    }

    webServer.send(200, "text/plain", "Stand-up started.");
}

void handleStandup() {
    startStandupRequest();
}

void handleContinuousGaitStart(WalkMode requestedMode, float pathTargetRightMm = kPathTargetSideMm) {
    if (legOperationIsActive()) {
        webServer.send(409, "text/plain", "Leg movement active.");
        return;
    }

    refreshPca9685Boards(true);
    if (!pcaState.ready || !pcaState.solderedReady) {
        webServer.send(503, "text/plain", "Both PCA9685 boards 0x40 and 0x41 must be connected.");
        return;
    }

    const bool turnRequested = requestedMode == WalkMode::TurnClockwise || requestedMode == WalkMode::TurnCounterClockwise || requestedMode == WalkMode::TurnClockwiseFast || requestedMode == WalkMode::TurnCounterClockwiseFast;
    const bool calculatedPathRequested = requestedMode == WalkMode::CalculatedPath;
    const bool finiteMotionRequested = turnRequested || calculatedPathRequested;

    const bool continuousRippleRequested = isContinuousRippleMode(requestedMode);
    const bool obstacleAvoidanceRequested = requestedMode == WalkMode::ContinuousRippleV6ObstacleAvoidance;
    const bool continuousRippleV6Requested = requestedMode == WalkMode::ContinuousRippleV6FullIk || obstacleAvoidanceRequested;

    const bool startingFromCenter = standState.stage == StandStage::WalkingPoseComplete && allLegCommandsMatchWalkingPose(); // starting point
    const bool resumingFromRear = !finiteMotionRequested && requestedMode == gaitState.walkMode && standState.stage == StandStage::WaveStopped && allLegCommandsMatchWaveRearPose();
    const bool resumingDualRipple = requestedMode == WalkMode::DualRipple && standState.stage == StandStage::WaveStopped && gaitState.walkMode == WalkMode::DualRipple && allLegCommandsKnown();
    const bool resumingContinuousRipple = !finiteMotionRequested && continuousRippleRequested && standState.stage == StandStage::WaveStopped && isContinuousRippleMode(gaitState.walkMode) && allLegCommandsMatchContinuousRippleStopPose();

    if (standState.mode != StandMode::FullRobot || (!startingFromCenter && !resumingFromRear && !resumingDualRipple && !resumingContinuousRipple)) {
        webServer.send(409, "text/plain", "Wrong pose.");
        return;
    }

    if (calculatedPathRequested) {
        calculateDiagonalPathPlan(kPathTargetForwardMm, pathTargetRightMm);
    }

    if (continuousRippleV6Requested && !validateContinuousRippleV6FullIk()) {
        webServer.send(500, "text/plain", standState.fault);
        return;
    }

    if (obstacleAvoidanceRequested && !obstacleNavigationCanStart(millis())) {
        webServer.send(503, "text/plain", "ToF data unavailable.");
        return;
    }

    cancelManualAim();
    cannonState.aimTrackingEnabled = false;
    aimPwm.setPWM(kAimServoChannel, 0, 4096);
    cannonState.appliedAimAction = AimServoAction::Off;
    cannonState.appliedAimCommand = -1;
    stopShootServo();
    stopTiltServo();
    stopChargeServo();

    standState.mode = StandMode::FullRobot;
    standState.fault[0] = '\0';
    gaitState.walkMode = requestedMode;

    if (!calculatedPathRequested) {
        gaitState.calculatedPathPhase = CalculatedPathPhase::None;
    } else if (pathPlanState.turnCycles > 0U) {
        gaitState.calculatedPathPhase = CalculatedPathPhase::TurnToTarget;
    } else {
        gaitState.calculatedPathPhase = CalculatedPathPhase::MoveToTarget;
    }

    if (!(resumingDualRipple && !resumingFromRear) && !resumingContinuousRipple) {
        resetActiveWalkGroup();
    }

    if (!resumingContinuousRipple) {
        gaitState.ripplePriming = continuousRippleRequested && startingFromCenter;
        gaitState.rippleFullPriming = false;
        gaitState.ripplePrimeStep = 0U;
    } else if (gaitState.ripplePriming) {
        advanceContinuousRipplePriming();
    }

    gaitState.waveStopRequested = false;
    gaitState.completedTurnCycles = 0;
    gaitState.completedCalculatedPathPhaseCycles = 0;

    const uint32_t gaitStartMs = millis();
    beginWaveStage((startingFromCenter || resumingContinuousRipple) ? StandStage::WaveLift : StandStage::WaveRecoveryLift, gaitStartMs);
    if (obstacleAvoidanceRequested) {
        startObstacleNavigation(gaitStartMs);
    }

    if (turnRequested) {
        const bool clockwise = clockwiseTurnActive();
        const bool fast = fastTurnModeActive();
        if (fast) {
            Serial.println(clockwise ? "Fast turn CW start" : "Fast turn CCW start");
            webServer.send( 200, "text/plain", clockwise
                    ? "Fast 180° clockwise started."
                    : "Fast 180° counterclockwise started.");
        } else {
            webServer.send( 200, "text/plain", clockwise ? "Calibrated 180-degree clockwise turn started" : "Calibrated 180-degree counter-clockwise turn started");
        }
        return;
    }

    if (calculatedPathRequested) {
        webServer.send(200, "text/plain", pathPlanState.targetRightMm < 0.0F ? "Forward-left path started." : "Forward-right path started.");
        return;
    }

    if (continuousRippleRequested) {
        Serial.println(continuousRippleV6Requested ? "V6 start" : "V4 start");
        if (obstacleAvoidanceRequested) {
            webServer.send(200, "text/plain", "Obstacle avoidance started.");
        } else if (continuousRippleV6Requested) {
            webServer.send(200, "text/plain", resumingContinuousRipple ? "V6 resumed." : "V6 started.");
        } else {
            webServer.send(200, "text/plain", resumingContinuousRipple ? "V4 resumed." : "V4 started.");
        }
        return;
    }

    const bool dualRipple = requestedMode == WalkMode::DualRipple;
    webServer.send( 200, "text/plain", dualRipple ? "Dual ripple started." : "Wave walk started.");
}

void handleWaveCycle() {
    handleContinuousGaitStart(WalkMode::SingleWave);
}

void handleDualRipple() {
    handleContinuousGaitStart(WalkMode::DualRipple);
}

void handleContinuousRippleV4DistributedStance() {
    handleContinuousGaitStart(WalkMode::ContinuousRippleV4DistributedStance);
}

void handleContinuousRippleV6FullIk() {
    handleContinuousGaitStart(WalkMode::ContinuousRippleV6FullIk);
}

void handleObstacleAvoidanceV6() {
    handleContinuousGaitStart(WalkMode::ContinuousRippleV6ObstacleAvoidance);
}

void handleFastTurnClockwise180() {
    handleContinuousGaitStart(WalkMode::TurnClockwiseFast);
}

void handleFastTurnCounterClockwise180() {
    handleContinuousGaitStart(WalkMode::TurnCounterClockwiseFast);
}

void handleCalculatedPathForwardLeft() {
    handleContinuousGaitStart(WalkMode::CalculatedPath, -kPathTargetSideMm);
}

void handleCalculatedPathForwardRight() {
    handleContinuousGaitStart(WalkMode::CalculatedPath, kPathTargetSideMm);
}

void handleWaveStop() {
    if (obstacleNavigationActive()) {
        requestObstacleNavigationStop();
        webServer.send(200, "text/plain", "Obstacle avoidance stop requested.");
        return;
    }

    if (!waveStageIsActive()) {
        webServer.send(409, "text/plain", "No movement active.");
        return;
    }

    if (gaitState.waveStopRequested) {
        webServer.send(200, "text/plain", "Stop already pending.");
        return;
    }

    gaitState.waveStopRequested = true;
    webServer.send(200, "text/plain", "Stop requested.");
}

void handleLegOutputsOff() {
    if (!pcaState.ready && !pcaState.solderedReady) {
        webServer.send(503, "text/plain", "Neither PCA9685 board is connected.");
        return;
    }

    cancelManualAim();
    cancelObstacleNavigation();
    disableAllLegOutputs();

    standState.stage = StandStage::OutputsOff;
    standState.mode = StandMode::None;
    standState.progress = 0.0F;
    standState.fault[0] = '\0';
    webServer.send(200, "text/plain", "Leg PWM disabled.");
}

} // namespace Hexapod
