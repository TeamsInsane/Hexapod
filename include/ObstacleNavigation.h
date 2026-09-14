#pragma once

#include <Arduino.h>

namespace Hexapod {
    bool obstacleNavigationActive();
    bool obstacleNavigationCanStart(uint32_t nowMs);
    const char *obstacleNavigationStateName();
    const char *obstacleNavigationReason();
    void startObstacleNavigation(uint32_t nowMs);
    void updateObstacleNavigation(uint32_t nowMs);
    void requestObstacleNavigationStop();
    void cancelObstacleNavigation();
} // namespace Hexapod
