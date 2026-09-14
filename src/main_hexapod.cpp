#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

#include "HexapodState.h"
#include "CannonPreparation.h"
#include "GaitController.h"
#include "ObstacleNavigation.h"
#include "CannonController.h"
#include "ToFSensors.h"
#include "WebInterface.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin(RobotConfig::kI2cSda, RobotConfig::kI2cScl);
    Hexapod::refreshPca9685Boards(true);
    Hexapod::beginToFSensors(Wire);

    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(Hexapod::kAccessPointName, Hexapod::kAccessPointPassword)) {
        Serial.println("ERROR: Wi-Fi access point failed to start");
        return;
    }

    Hexapod::setupWebServer();

    Serial.println();
    Serial.printf("Wi-Fi: %s\n", Hexapod::kAccessPointName);
    Serial.printf("Password: %s\n", Hexapod::kAccessPointPassword);
    Serial.printf("Open: http://%s/\n", WiFi.softAPIP().toString().c_str());
}

void loop() {
    Hexapod::updateToFSensors(millis());
    Hexapod::webServer.handleClient();
    Hexapod::refreshPca9685Boards();
    Hexapod::updateAimServo();
    Hexapod::updateShootServo();
    Hexapod::updateTiltServo();
    Hexapod::updateChargeServo();
    Hexapod::updateCannonPreparation(millis());
    Hexapod::updateLegOperation();
    Hexapod::updateObstacleNavigation(millis());
    delay(2);
}
