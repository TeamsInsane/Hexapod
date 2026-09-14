#include "HexapodState.h"

namespace Hexapod {

WebServer webServer(80);
Adafruit_PWMServoDriver aimPwm(RobotConfig::kPcaDefaultAddress);
Adafruit_PWMServoDriver solderedPwm(RobotConfig::kPcaSolderedAddress);
AimState aimState = {};
PcaState pcaState = {};
CannonState cannonState = {};
StandState standState = {};
GaitState gaitState = {};
PathPlanState pathPlanState = {};

} // namespace Hexapod
