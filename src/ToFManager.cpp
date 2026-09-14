#include "ToFManager.h"

#include <stdio.h>

namespace {

bool i2cAddressResponds(TwoWire &wire, uint8_t address) {
    wire.beginTransmission(address);
    return wire.endTransmission() == 0;
}

} // namespace

bool ToFManager::begin(TwoWire &wire) {
    wire_ = &wire;
    bool anyInitialized = false;
    fault_[0] = '\0';

    if (!i2cAddressResponds(*wire_, RobotConfig::kPca9548aAddress)) {
        setFault("PCA9548A mux not found");
        return false;
    }

    for (uint8_t index = 0; index < RobotConfig::kToFSensorCount; ++index) {
        sensors_[index].sampleCount = 0;
        sensors_[index].sampleIndex = 0;
        sensors_[index].initialized = false;
        sensors_[index].staleSinceMs = 0;
        sensors_[index].reading = {0, 0, 0, false, false};

        if (initSensor(index)) {
            anyInitialized = true;
        }
    }

    if (!anyInitialized) {
        setFault("No VL53L0X sensors initialized");
    }

    return anyInitialized;
}

void ToFManager::update(uint32_t nowMs) {
    const uint8_t index = nextUpdateIndex_;
    SensorState &state = sensors_[index];
    const RobotConfig::ToFSensorConfig &config = RobotConfig::kToFSensors[index];
    bool staleNow = !state.initialized;

    if (state.initialized && selectChannel(config.muxChannel)) {
        if ((state.sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07U) != 0U) {
            const uint16_t rawMm = state.sensor.readRangeContinuousMillimeters();
            const bool timedOut = state.sensor.timeoutOccurred();

            // VL53L0X commonly reports 8190/8191 mm when no valid
            const bool valid = !timedOut && rawMm > 0U && rawMm < 8190U;
            recordSample(index, rawMm, valid, timedOut, nowMs);

            staleNow = timedOut;
        } else if (nowMs - state.reading.timestampMs > RobotConfig::kSensorStaleTimeoutMs) {
            state.reading.valid = false;
            state.reading.timedOut = true;
            staleNow = true;
        }
    } else if (state.initialized) {
        state.reading.valid = false;
        state.reading.timedOut = true;
        staleNow = true;
    }

    if (!staleNow) {
        state.staleSinceMs = 0;
    } else if (state.staleSinceMs == 0U) {
        state.staleSinceMs = nowMs;
    } else if (nowMs - state.staleSinceMs >= RobotConfig::kSensorReinitializeDelayMs) {
        state.staleSinceMs = nowMs;
        Serial.printf("VL53L0X %s on CH%u stale for 5 s, reinitializing\n", config.name, static_cast<unsigned int>(config.muxChannel));
        if (initSensor(index)) {
            bool allInitialized = true;
            for (uint8_t sensorIndex = 0; sensorIndex < RobotConfig::kToFSensorCount; ++sensorIndex) {
                allInitialized = allInitialized && sensors_[sensorIndex].initialized;
            }

            if (allInitialized) {
                fault_[0] = '\0';
            }

            Serial.printf("VL53L0X %s reinitialized\n", config.name);
        } else {
            Serial.printf("VL53L0X %s reinitialization failed %s\n", config.name, fault_);
        }
    }

    nextUpdateIndex_ = (index + 1U) % RobotConfig::kToFSensorCount;
}

const DistanceReading &ToFManager::readingByIndex(uint8_t index) const {
    return sensors_[index].reading;
}

bool ToFManager::initializedByIndex(uint8_t index) const {
    return sensors_[index].initialized;
}

bool ToFManager::selectChannel(uint8_t channel) {
    wire_->beginTransmission(RobotConfig::kPca9548aAddress);
    wire_->write(static_cast<uint8_t>(1U << channel));
    return wire_->endTransmission() == 0;
}

bool ToFManager::initSensor(uint8_t index) {
    SensorState &state = sensors_[index];
    const RobotConfig::ToFSensorConfig &config = RobotConfig::kToFSensors[index];
    state.initialized = false;

    if (!selectChannel(config.muxChannel)) {
        snprintf(fault_, sizeof(fault_), "Mux select failed for %s", config.name);
        return false;
    }

    state.sensor.setBus(wire_);
    state.sensor.setTimeout(50);

    if (!state.sensor.init()) {
        snprintf(fault_, sizeof(fault_), "VL53L0X init failed for %s", config.name);
        return false;
    }

    state.sensor.setSignalRateLimit(0.1F);
    state.sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    state.sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    state.sensor.setMeasurementTimingBudget(50000);
    state.sensor.startContinuous(50);
    state.sampleCount = 0;
    state.sampleIndex = 0;
    state.reading = {0, 0, 0, false, false};
    state.staleSinceMs = 0;
    state.initialized = true;
    return true;
}

void ToFManager::recordSample(uint8_t index, uint16_t rawMm, bool valid, bool timedOut, uint32_t nowMs) {
    SensorState &state = sensors_[index];
    state.reading.rawMm = rawMm;
    state.reading.timestampMs = nowMs;
    state.reading.timedOut = timedOut;

    if (!valid) {
        state.reading.valid = false;
        return;
    }

    state.samples[state.sampleIndex] = rawMm;
    state.sampleIndex = (state.sampleIndex + 1) % 5;
    if (state.sampleCount < 5) {
        ++state.sampleCount;
    }

    state.reading.filteredMm = medianSample(state);
    state.reading.valid = state.sampleCount >= 3;
}

uint16_t ToFManager::medianSample(const SensorState &state) const {
    uint16_t sorted[5] = {};
    for (uint8_t index = 0; index < state.sampleCount; ++index) {
        sorted[index] = state.samples[index];
    }

    for (uint8_t i = 0; i < state.sampleCount; ++i) {
        for (uint8_t j = i + 1; j < state.sampleCount; ++j) {
            if (sorted[j] < sorted[i]) {
                const uint16_t tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    return sorted[state.sampleCount / 2];
}

void ToFManager::setFault(const char *message) {
    snprintf(fault_, sizeof(fault_), "%s", message);
}
