#pragma once

#include "RobotConfig.h"
#include "RobotTypes.h"

class Kinematics {
public:
    JointAnglesDeg solveLeg(LegId leg, const Vec3 &footBodyMm, char *faultBuffer, size_t faultBufferSize) const;
    FootTarget solveForwardLeg(LegId leg, const JointAnglesDeg &angles, char *faultBuffer, size_t faultBufferSize) const;
    JointAnglesDeg servoToModelAngles(LegId leg, const ServoAnglesDeg &servoAngles, char *faultBuffer, size_t faultBufferSize) const;
    ServoAnglesDeg modelToServoAngles(LegId leg, const JointAnglesDeg &modelAngles, char *faultBuffer, size_t faultBufferSize) const;

private:
    Vec3 bodyToLegFrame(uint8_t legIndex, const Vec3 &footBodyMm) const;
    Vec3 legToBodyFrame(uint8_t legIndex, const Vec3 &footLegMm) const;
};
