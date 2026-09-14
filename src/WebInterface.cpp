#include <Arduino.h>

#include "HexapodState.h"
#include "AimMessageHandler.h"
#include "WebCannon.h"
#include "WebContent.h"
#include "WebFirmwareUpdate.h"
#include "WebInterface.h"
#include "WebMovement.h"
#include "WebStatus.h"

namespace Hexapod {

void setupWebServer() {
    webServer.on("/", HTTP_GET, []() {
        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send_P(200, "text/html", kIndexHtml);
    });

    webServer.on("/status", HTTP_GET, sendStatus);

    webServer.on("/api/aim", HTTP_POST, handleAimMessage);
    webServer.on("/api/aim/lost", HTTP_POST, handleAimLost);

    webServer.on("/api/standup", HTTP_POST, handleStandup); // Stand up
    webServer.on("/api/legs/wave-cycle", HTTP_POST, handleWaveCycle); // One-leg wave walk
    webServer.on("/api/legs/dual-ripple", HTTP_POST, handleDualRipple); // Dual-ripple walk with push
    webServer.on("/api/legs/continuous-ripple-v4", HTTP_POST, handleContinuousRippleV4DistributedStance); // V4 ripple walk
    webServer.on("/api/legs/continuous-ripple-v6", HTTP_POST, handleContinuousRippleV6FullIk); //  V6 ripple walk
    webServer.on("/api/legs/obstacle-v6", HTTP_POST, handleObstacleAvoidanceV6); //  V6 obstacle avoidance
    webServer.on("/api/legs/turn-180-fast/cw", HTTP_POST, handleFastTurnClockwise180); //  Test fast 180° clockwise turn
    webServer.on("/api/legs/turn-180-fast/ccw", HTTP_POST, handleFastTurnCounterClockwise180); // Test fast 180° counterclockwise turn
    webServer.on("/api/legs/path-forward-left", HTTP_POST, handleCalculatedPathForwardLeft); // 30 cm forward + 30 cm left
    webServer.on("/api/legs/path-forward-right", HTTP_POST, handleCalculatedPathForwardRight); //   30 cm forward + 30 cm right
    webServer.on("/api/legs/wave-stop", HTTP_POST, handleWaveStop); // Stop walking
    webServer.on("/api/legs/off", HTTP_POST, handleLegOutputsOff); // Turn leg servos off

    webServer.on("/api/cannon/prepare/start", HTTP_POST, handleCannonPreparationStart); // Aim and calculate
    webServer.on("/api/cannon/prepare/auto", HTTP_POST, handleAutomaticCannonPreparationStart); // Full automatic setup; firing stays manual
    webServer.on("/api/cannon/prepare/confirm", HTTP_POST, handleCannonPreparationConfirm); // Confirm plan and lower cannon (1200 ms)
    webServer.on("/api/cannon/prepare/tilt-home/confirm", HTTP_POST, handleCannonPreparationConfirmTiltHome); // Cannon is fully down
    webServer.on("/api/cannon/prepare/tilt-up/confirm", HTTP_POST, handleCannonPreparationConfirmTiltUp); // Confirm barrel angle before charging
    webServer.on("/api/cannon/prepare/charge/add-200ms", HTTP_POST, handleCannonPreparationAddChargeStep); // Add 200 ms pull
    webServer.on("/api/cannon/prepare/confirm-charged-unwind", HTTP_POST, handleCannonPreparationConfirmChargedAndUnwind); // Charge is ready — unwind
    webServer.on("/api/cannon/prepare/cancel", HTTP_POST, handleCannonPreparationCancel); // Cancel setup

    webServer.on("/api/servo/start", HTTP_POST, handleServoStart); // Start tracking
    webServer.on("/api/servo/stop", HTTP_POST, handleServoStop); // Stop tracking
    webServer.on("/api/servo/nudge/left", HTTP_POST, handleAimNudgeLeft); // Turn left (250 ms)
    webServer.on("/api/servo/nudge/right", HTTP_POST, handleAimNudgeRight); // Turn right (250 ms)
    webServer.on("/api/shoot", HTTP_POST, handleShoot); // Shoot and reset (500 + 500 ms)
    webServer.on("/api/tilt/up", HTTP_POST, handleTiltUp); // Tilt up (200 ms)
    webServer.on("/api/tilt/down", HTTP_POST, handleTiltDown); // Tilt down (200 ms)
    webServer.on("/api/charge/charge-half", HTTP_POST, handleChargeHalfSecond); //Charge +500 ms
    webServer.on("/api/charge/p2", HTTP_POST, handleChargeP2); // Charge P2 (6000 ms)
    webServer.on("/api/charge/uncharge-half", HTTP_POST, handleUnchargeHalfSecond); // Release charge (500 ms)
    webServer.on("/api/charge/unwind-recorded", HTTP_POST, handleUnwindRecordedCharge); //Unwind charge
    webServer.on("/api/charge/mark-home", HTTP_POST, handleMarkChargeHome); // Confirm charge is at HOME
    webServer.on("/api/charge/stop", HTTP_POST, handleChargeStop); // Stop charge

    webServer.on("/update", HTTP_GET, []() {
        webServer.send_P(200, "text/html", kUpdateHtml);
    });

    webServer.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);
    webServer.on("/firmware.bin", HTTP_GET, handleFirmwareDownload);

    webServer.onNotFound([]() {
        webServer.send(404, "text/plain", "Not found");
    });

    webServer.begin();
}

} // namespace Hexapod
