#include <Arduino.h>
#include <stdio.h>

#include "HexapodState.h"
#include "CannonPreparation.h"
#include "GaitController.h"
#include "CannonController.h"
#include "WebCannon.h"
#include "WebStatus.h"


namespace Hexapod {

bool rejectManualCannonDuringPreparation() {
    if (!cannonPreparationActive()) {
        return false;
    }

    webServer.send(409, "text/plain", "Cannon setup active.");
    return true;
}

bool rejectCommandDuringStandup() {
    if (!legOperationIsActive()) {
        return false;
    }

    webServer.send(409, "text/plain", "Leg movement active.");
    return true;
}

void startCannonPreparationRequest(bool fullyAutomatic) {
    refreshPca9685Boards(true);
    if (!pcaState.ready) {
        webServer.send(503, "text/plain", "PCA9685 0x40 not connected");
        return;
    }

    const bool started = fullyAutomatic
        ? startAutomaticCannonPreparation(millis())
        : startCannonPreparation(millis());
    if (!started) {
        webServer.send(409, "text/plain", "Cannon is busy.");
        return;
    }

    webServer.send(200, "text/plain", fullyAutomatic
        ? "Auto setup started."
        : "Aim capture started.");
}

void handleCannonPreparationStart() {
    startCannonPreparationRequest(false);
}

void handleAutomaticCannonPreparationStart() {
    startCannonPreparationRequest(true);
}

void handleCannonPreparationConfirm() {
    if (!confirmCannonPreparationPlan()) {
        webServer.send(409, "text/plain", "No plan ready.");
        return;
    }

    webServer.send(200, "text/plain", "Plan confirmed.");
}

void handleCannonPreparationConfirmTiltHome() {
    if (!confirmCannonPreparationTiltFullyDown()) {
        webServer.send(409, "text/plain", "Tilt confirmation unavailable.");
        return;
    }

    webServer.send(200, "text/plain", "Tilt confirmed.");
}

void handleCannonPreparationConfirmTiltUp() {
    if (!confirmCannonPreparationTiltUp()) {
        webServer.send(409, "text/plain", "Tilt confirmation unavailable.");
        return;
    }

    webServer.send(200, "text/plain", "Tilt confirmed. Charging started.");
}

void handleCannonPreparationAddChargeStep() {
    if (!addCannonPreparationChargeStep()) {
        webServer.send(409, "text/plain", "Charge adjustment unavailable.");
        return;
    }

    webServer.send(200, "text/plain", "Adding 200 ms.");
}

void handleCannonPreparationConfirmChargedAndUnwind() {
    if (!confirmCannonPreparationChargedAndUnwind()) {
        webServer.send(409, "text/plain", "Unwind unavailable.");
        return;
    }

    webServer.send(200, "text/plain", "Unwinding charge.");
}

void handleCannonPreparationCancel() {
    cancelCannonPreparation("cancelled from website");
    webServer.send(200, "text/plain", "Setup cancelled.");
}

void handleServoStart() {
    if (rejectManualCannonDuringPreparation() || rejectCommandDuringStandup()) {
        return;
    }

    refreshPca9685Boards(true);
    if (!pcaState.ready) {
        cannonState.aimTrackingEnabled = false;
        webServer.send(503, "text/plain", "Non-soldered PCA9685 0x40 is not connected.");
        return;
    }

    cancelManualAim();
    cannonState.aimTrackingEnabled = true;
    updateAimServo();
    sendAimTrackingControlStatus();
}

void handleServoStop() {
    cancelCannonPreparation("stopped with the normal servo stop button");
    cancelManualAim();
    cannonState.aimTrackingEnabled = false;

    if (pcaState.ready) {
        aimPwm.setPWM(kAimServoChannel, 0, 4096);
    }

    cannonState.appliedAimAction = AimServoAction::Off;
    cannonState.appliedAimCommand = -1;
    sendAimTrackingControlStatus();
}

void handleAimNudge(AimServoAction action) {
    if (rejectManualCannonDuringPreparation() || rejectCommandDuringStandup()) {
        return;
    }
    refreshPca9685Boards(true);
    if (!pcaState.ready) {
        webServer.send(503, "text/plain", "Non-soldered PCA9685 0x40 is not connected.");
        return;
    }

    cancelManualAim();
    cannonState.aimTrackingEnabled = false;
    cannonState.manualAimActive = true;
    cannonState.manualAimAction = action;
    cannonState.manualAimStopAtMs = millis() + kManualAimRunDurationMs;
    applyAimServoAction(action);

    char message[112] = "";
    snprintf(message, sizeof(message), "Aim %s: %lu ms.", aimServoActionName(action), static_cast<unsigned long>(kManualAimRunDurationMs));
    webServer.send(200, "text/plain", message);
}

void handleAimNudgeLeft() {
    handleAimNudge(AimServoAction::Left);
}

void handleAimNudgeRight() {
    handleAimNudge(AimServoAction::Right);
}

void startShootRequest(ShootPhase firstPhase, bool returnAfterForward, const char *responseMessage) {
    if (rejectCommandDuringStandup()) {
        return;
    }

    if (cannonPreparationActive() && (!returnAfterForward || !cannonPreparationReady())) {
        webServer.send(409, "text/plain", "Cannon setup active.");
        return;
    }

    refreshPca9685Boards(true);
    if (!pcaState.ready) {
        webServer.send(503, "text/plain", "Non-soldered PCA9685 0x40 is not connected.");
        return;
    }

    if (shootServoIsActive()) {
        webServer.send(409, "text/plain", "Shoot servo busy.");
        return;
    }

    cancelManualAim();
    cannonState.shootReturnAfterForward = returnAfterForward;
    startShootServoPhase(firstPhase);
    if (cannonPreparationActive()) {
        cannonPreparationShotStarted();
    }

    webServer.send(200, "text/plain", responseMessage);
}

void handleShoot() {
    startShootRequest( ShootPhase::Forward, true, "Shoot sequence started" );
}

void startTiltRequest(TiltAction action, uint32_t durationMs) {
    if (rejectManualCannonDuringPreparation() || rejectCommandDuringStandup()) {
        return;
    }

    refreshPca9685Boards(true);
    if (!pcaState.ready) {
        webServer.send(503, "text/plain", "Non-soldered PCA9685 0x40 is not connected");
        return;
    }

    if (tiltServoIsActive()) {
        webServer.send(409, "text/plain", "Tilt servo busy.");
        return;
    }

    cancelManualAim();
    startTiltServo(action, durationMs);
    char response[96] = "";
    snprintf( response, sizeof(response), "Cannon %s test started for %lu ms.", tiltActionName(action), static_cast<unsigned long>(durationMs) );
    webServer.send( 200, "text/plain", response );
}

void handleTiltUp() {
    startTiltRequest(TiltAction::Up, kTiltRunDurationMs);
}

void handleTiltDown() {
    startTiltRequest(TiltAction::Down, kTiltRunDurationMs);
}

void startChargeRequest(ChargeAction action, uint32_t durationMs, const char *description, bool requireEstimatedHome = false) {
    if (rejectManualCannonDuringPreparation() || rejectCommandDuringStandup()) {
        return;
    }

    refreshPca9685Boards(true);
    if (!pcaState.ready) {
        webServer.send(503, "text/plain", "Non-soldered PCA9685 0x40 is not connected.");
        return;
    }

    if (chargeServoIsActive()) {
        webServer.send(409, "text/plain", "Charge servo busy.");
        return;
    }

    if (action == ChargeAction::Charge && requireEstimatedHome && cannonState.estimatedWoundDurationMs > 0U) {
        webServer.send( 409, "text/plain", "The P2 preset must start from HOME");
        return;
    }

    if (action == ChargeAction::Charge && durationMs > kMaximumChargeDurationMs - cannonState.estimatedWoundDurationMs) {
        webServer.send(422, "text/plain", "Charge limit exceeded.");
        return;
    }

    if (durationMs == 0U || durationMs > (action == ChargeAction::Charge ? kMaximumChargeDurationMs : kMaximumUnchargeDurationMs)) {
        webServer.send(422, "text/plain", "Invalid charge duration.");
        return;
    }

    cancelManualAim();
    startChargeServo(action, durationMs);
    char message[180] = "";
    snprintf(message, sizeof(message), "%s: %lu ms.", description, static_cast<unsigned long>(durationMs));
    webServer.send(200, "text/plain", message);
}

void handleChargeHalfSecond() {
    startChargeRequest(ChargeAction::Charge, kChargeFineStepDurationMs, "Half-second charge step");
}

void handleChargeP2() {
    startChargeRequest(ChargeAction::Charge, kChargeP2DurationMs, "Stable P2 charge", true);
}

void handleUnchargeHalfSecond() {
    startChargeRequest(ChargeAction::Uncharge, kChargeFineStepDurationMs, "Half-second uncharge step");
}

void handleUnwindRecordedCharge() {
    if (cannonState.estimatedWoundDurationMs == 0U) {
        webServer.send(409, "text/plain", "Nothing to unwind.");
        return;
    }

    startChargeRequest(ChargeAction::Uncharge, cannonState.estimatedWoundDurationMs + kRecordedUnwindExtraDurationMs, "Unwind recorded pull plus 0.5-second");
}

void handleMarkChargeHome() {
    if (rejectManualCannonDuringPreparation() || rejectCommandDuringStandup()) {
        return;
    }

    if (chargeServoIsActive()) {
        webServer.send(409, "text/plain", "Stop charge before setting HOME.");
        return;
    }

    markChargeMechanismHome();
    webServer.send( 200, "text/plain", "Charge HOME set.");
}

void handleChargeStop() {
    if (cannonPreparationActive()) {
        cancelCannonPreparation("charge stop requested from website");
    } else {
        stopChargeServo();
    }

    webServer.send(200, "text/plain", "Charge servo stopped.");
}

} // namespace Hexapod
