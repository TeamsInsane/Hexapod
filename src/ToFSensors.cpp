#include "ToFSensors.h"

#include "ToFManager.h"

namespace Hexapod {
namespace {

ToFManager tofManager;
bool sensorsStarted = false;
uint32_t lastSensorUpdateMs = 0;

} // namespace

bool beginToFSensors(TwoWire &wire) {
    sensorsStarted = tofManager.begin(wire);

    if (sensorsStarted) {
        Serial.println("VL53L0X sensors ready: front=CH1, left=CH7, right=CH0");
    } else {
        Serial.printf("VL53L0X sensors unavailable: %s\n", tofManager.fault());
    }

    return sensorsStarted;
}

void updateToFSensors(uint32_t nowMs) {
    if (!sensorsStarted || nowMs - lastSensorUpdateMs < RobotConfig::kSensorUpdateIntervalMs) {
        return;
    }

    lastSensorUpdateMs = nowMs;
    tofManager.update(nowMs);
}

bool tofSensorsStarted() {
    return sensorsStarted;
}

const DistanceReading &tofSensorReading(uint8_t index) {
    return tofManager.readingByIndex(index);
}

bool tofSensorInitialized(uint8_t index) {
    return tofManager.initializedByIndex(index);
}

const char *tofSensorFault() {
    return tofManager.fault();
}

} // namespace Hexapod
