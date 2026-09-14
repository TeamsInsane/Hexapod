#pragma once

#include <VL53L0X.h>
#include "RobotConfig.h"
#include "RobotTypes.h"

class ToFManager {
public:
    bool begin(TwoWire &wire);
    void update(uint32_t nowMs);
    const DistanceReading &readingByIndex(uint8_t index) const;
    bool initializedByIndex(uint8_t index) const;
    const char *fault() const { return fault_; }

private:
    struct SensorState {
        VL53L0X sensor;
        DistanceReading reading;
        uint16_t samples[5];
        uint8_t sampleCount;
        uint8_t sampleIndex;
        bool initialized;
        uint32_t staleSinceMs;
    };

    bool selectChannel(uint8_t channel);
    bool initSensor(uint8_t index);
    void recordSample(uint8_t index, uint16_t rawMm, bool valid, bool timedOut, uint32_t nowMs);
    uint16_t medianSample(const SensorState &state) const;
    void setFault(const char *message);

    TwoWire *wire_ = nullptr;
    SensorState sensors_[RobotConfig::kToFSensorCount] = {};
    uint8_t nextUpdateIndex_ = 0;
    char fault_[96] = "";
};
