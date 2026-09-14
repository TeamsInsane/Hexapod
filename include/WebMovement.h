#pragma once

namespace Hexapod {
    void handleStandup();
    void handleWaveCycle();
    void handleDualRipple();
    void handleContinuousRippleV4DistributedStance();
    void handleContinuousRippleV6FullIk();
    void handleObstacleAvoidanceV6();
    void handleFastTurnClockwise180();
    void handleFastTurnCounterClockwise180();
    void handleCalculatedPathForwardLeft();
    void handleCalculatedPathForwardRight();
    void handleWaveStop();
    void handleLegOutputsOff();
} // namespace Hexapod
