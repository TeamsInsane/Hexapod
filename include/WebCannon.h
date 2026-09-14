#pragma once

namespace Hexapod {
    void handleCannonPreparationStart();
    void handleAutomaticCannonPreparationStart();
    void handleCannonPreparationConfirm();
    void handleCannonPreparationConfirmTiltHome();
    void handleCannonPreparationConfirmTiltUp();
    void handleCannonPreparationAddChargeStep();
    void handleCannonPreparationConfirmChargedAndUnwind();
    void handleCannonPreparationCancel();
    void handleServoStart();
    void handleServoStop();
    void handleAimNudgeLeft();
    void handleAimNudgeRight();
    void handleShoot();
    void handleTiltUp();
    void handleTiltDown();
    void handleChargeHalfSecond();
    void handleChargeP2();
    void handleUnchargeHalfSecond();
    void handleUnwindRecordedCharge();
    void handleMarkChargeHome();
    void handleChargeStop();
} // namespace Hexapod
