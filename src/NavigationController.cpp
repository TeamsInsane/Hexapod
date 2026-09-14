#include <Arduino.h>

#include "HexapodState.h"
#include "NavigationController.h"

namespace Hexapod {

void calculateDiagonalPathPlan(float targetForwardMm, float targetRightMm) {
    // Enačbe diplomske naloge (4.24), (4.25) in (4.28)–(4.33):
    const float degreesPerTurnCycle = kMeasuredTurnDegrees / kMeasuredTurnCycleCount; // 4.28: 15 degrees turn per cycle
    const float mmPerCoxaStepDegree = kMeasuredForwardMm / (kMeasuredForwardCycleCount * static_cast<float>(kForwardCoxaStepDegrees)); // 4.24: 2,89mm for each degree
    const float requestedHeadingDegrees = atan2f(targetRightMm, targetForwardMm) * (180.0F / PI); // 4.29: Degrees to turn to target

    pathPlanState.targetForwardMm = targetForwardMm;
    pathPlanState.targetRightMm = targetRightMm;
    pathPlanState.turnCycles = static_cast<uint8_t>(constrain(static_cast<int>(lroundf(fabsf(requestedHeadingDegrees) / degreesPerTurnCycle)), 0, 255)); //4.30: How much turn cycles

    const float requiredStepDegrees = hypotf(targetForwardMm, targetRightMm) / mmPerCoxaStepDegree; // 4.29 in 4.31: How much degrees to reach target

    const int fullCycles = static_cast<int>(floorf(requiredStepDegrees / static_cast<float>(kForwardCoxaStepDegrees))); // How much forward cycles
    int remainingStepDegrees = static_cast<int>(lroundf(requiredStepDegrees - static_cast<float>(fullCycles * kForwardCoxaStepDegrees))); //How much left

    pathPlanState.forwardCycles = static_cast<uint8_t>(constrain(fullCycles + (remainingStepDegrees > 0 ? 1 : 0), 1, 255));
    pathPlanState.finalCoxaStepDegrees = remainingStepDegrees > 0 ? remainingStepDegrees : kForwardCoxaStepDegrees;

    const float headingSign = requestedHeadingDegrees < 0.0F ? -1.0F : 1.0F;
    pathPlanState.headingDegrees = headingSign * static_cast<float>(pathPlanState.turnCycles) * degreesPerTurnCycle;

    // 4.25 in 4.32
    pathPlanState.distanceMm = (static_cast<float>(pathPlanState.forwardCycles - 1U) * static_cast<float>(kForwardCoxaStepDegrees) + static_cast<float>(pathPlanState.finalCoxaStepDegrees)) * mmPerCoxaStepDegree;

    //4.33
    const float headingRadians = pathPlanState.headingDegrees * (PI / 180.0F);
    pathPlanState.predictedForwardMm = pathPlanState.distanceMm * cosf(headingRadians);
    pathPlanState.predictedRightMm = pathPlanState.distanceMm * sinf(headingRadians);
}


} // namespace Hexapod
