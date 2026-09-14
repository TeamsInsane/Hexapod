#include "Kinematics.h"

#include <math.h>
#include <stdio.h>

namespace {

constexpr float kDegToRad = PI / 180.0F;
constexpr float kRadToDeg = 180.0F / PI;
constexpr float kEpsilon = 0.001F;

float clampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

bool finiteVec(const Vec3 &value) {
    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

bool finiteModelAngles(const JointAnglesDeg &angles) {
    return angles.valid && isfinite(angles.coxa) && isfinite(angles.femur) && isfinite(angles.tibia);
}

bool finiteServoAngles(const ServoAnglesDeg &angles) {
    return angles.valid && isfinite(angles.coxa) && isfinite(angles.femur) && isfinite(angles.tibia);
}

} // namespace

JointAnglesDeg Kinematics::solveLeg(LegId leg, const Vec3 &footBodyMm, char *faultBuffer, size_t faultBufferSize) const {
    JointAnglesDeg result = {0.0F, 0.0F, 0.0F, false};
    const uint8_t legIndex = static_cast<uint8_t>(leg);

    if (!finiteVec(footBodyMm)) {
        snprintf(faultBuffer, faultBufferSize, "IK non-finite target leg %u", legIndex);
        return result;
    }

    const Vec3 local = bodyToLegFrame(legIndex, footBodyMm);
    // Diplomska enačbe (4.10)-(4.14): local 3-DOF inverse kinematics.
    const float coxaRad = atan2f(local.y, local.x); //4.10
    const float planarMm = sqrtf(local.x * local.x + local.y * local.y) - RobotConfig::kCoxaLengthMm; //4.10
    const float verticalMm = local.z;
    const float reachSquaredMm = planarMm * planarMm + verticalMm * verticalMm;
    const float reachMm = sqrtf(reachSquaredMm); //4.11

    //4.12
    if (reachMm < kEpsilon || reachMm > RobotConfig::kFemurLengthMm + RobotConfig::kTibiaLengthMm || reachMm < fabsf(RobotConfig::kFemurLengthMm - RobotConfig::kTibiaLengthMm)) {
        snprintf(faultBuffer, faultBufferSize, "IK unreachable leg %u target %.1f %.1f %.1f", legIndex, footBodyMm.x, footBodyMm.y, footBodyMm.z);
        return result;
    }

    //4.13 -> -1, 1
    const float tibiaCos = clampFloat( (reachSquaredMm - RobotConfig::kFemurLengthMm * RobotConfig::kFemurLengthMm - RobotConfig::kTibiaLengthMm * RobotConfig::kTibiaLengthMm) /
            (2.0F * RobotConfig::kFemurLengthMm * RobotConfig::kTibiaLengthMm), -1.0F,1.0F);

    //4.14
    const float tibiaRad = -acosf(tibiaCos); //0, -PI
    const float femurRad = atan2f(verticalMm, planarMm) - atan2f(RobotConfig::kTibiaLengthMm * sinf(tibiaRad), RobotConfig::kFemurLengthMm + RobotConfig::kTibiaLengthMm * cosf(tibiaRad));

    result.coxa = coxaRad * kRadToDeg;
    result.femur = femurRad * kRadToDeg;
    result.tibia = tibiaRad * kRadToDeg;
    result.valid = true;
    return result;
}

// Computes a foot position from calibrated model joint angles
FootTarget Kinematics::solveForwardLeg(LegId leg, const JointAnglesDeg &angles, char *faultBuffer, size_t faultBufferSize) const {
    FootTarget result = {{0.0F, 0.0F, 0.0F}, false};
    const uint8_t legIndex = static_cast<uint8_t>(leg);
    if (!finiteModelAngles(angles)) {
        snprintf(faultBuffer, faultBufferSize, "FK non-finite angles leg %u", legIndex);
        return result;
    }

    // Dimplomska enačbe (2.1), (4.8), and (4.9): forward kinematics.
    const float coxaRad = angles.coxa * kDegToRad;
    const float femurRad = angles.femur * kDegToRad;
    const float tibiaRad = angles.tibia * kDegToRad;

    const float planarMm = RobotConfig::kFemurLengthMm * cosf(femurRad) + RobotConfig::kTibiaLengthMm * cosf(femurRad + tibiaRad);
    const float radialMm = RobotConfig::kCoxaLengthMm + planarMm;

    const Vec3 local = {
        radialMm * cosf(coxaRad),
        radialMm * sinf(coxaRad),
        RobotConfig::kFemurLengthMm * sinf(femurRad) + RobotConfig::kTibiaLengthMm * sinf(femurRad + tibiaRad),
    };

    result.bodyPositionMm = legToBodyFrame(legIndex, local);
    result.valid = true;
    return result;
}

// Converts physical servo commands into the joint angles used by the geometric IK/FK model
JointAnglesDeg Kinematics::servoToModelAngles(LegId leg, const ServoAnglesDeg &servoAngles, char *faultBuffer, size_t faultBufferSize) const {
    JointAnglesDeg result = {0.0F, 0.0F, 0.0F, false};
    const uint8_t legIndex = static_cast<uint8_t>(leg);
    if (!finiteServoAngles(servoAngles)) {
        snprintf(faultBuffer, faultBufferSize, "Calibration non-finite servo angles leg %u", legIndex);
        return result;
    }

    const float raw[RobotConfig::kJointCount] = {servoAngles.coxa, servoAngles.femur, servoAngles.tibia};
    float model[RobotConfig::kJointCount] = {};
    for (uint8_t joint = 0; joint < RobotConfig::kJointCount; ++joint) {
        const RobotConfig::JointCalibration &cal = RobotConfig::kJointCalibration[legIndex][joint];
        if (raw[joint] < cal.minDeg || raw[joint] > cal.maxDeg) {
            snprintf(faultBuffer, faultBufferSize, "Calibration servo limit leg %u joint %u", legIndex, joint);
            return result;
        }

        // Diplomska enačba 4.4
        model[joint] = cal.offsetDeg + cal.direction * (raw[joint] - cal.neutralDeg);
    }

    result = {model[0], model[1], model[2], true};
    return result;
}

ServoAnglesDeg Kinematics::modelToServoAngles(LegId leg, const JointAnglesDeg &modelAngles, char *faultBuffer, size_t faultBufferSize) const {
    ServoAnglesDeg result = {0.0F, 0.0F, 0.0F, false};
    const uint8_t legIndex = static_cast<uint8_t>(leg);
    if (!finiteModelAngles(modelAngles)) {
        snprintf(faultBuffer, faultBufferSize, "Calibration non-finite model angles leg %u", legIndex);
        return result;
    }

    const float model[RobotConfig::kJointCount] = {
        modelAngles.coxa, modelAngles.femur, modelAngles.tibia};
    float servo[RobotConfig::kJointCount] = {};
    for (uint8_t joint = 0; joint < RobotConfig::kJointCount; ++joint) {
        const RobotConfig::JointCalibration &cal = RobotConfig::kJointCalibration[legIndex][joint];
        // Enačba diplomske (4.5)
        servo[joint] = cal.neutralDeg + (model[joint] - cal.offsetDeg) / cal.direction;
        if (servo[joint] < cal.minDeg || servo[joint] > cal.maxDeg) {
            snprintf(faultBuffer, faultBufferSize, "Calibration model limit leg %u joint %u", legIndex, joint);
            return result;
        }
    }

    result = {servo[0], servo[1], servo[2], true};
    return result;
}

// Dimplomska enačba (4.1): Lp = Rz(-gamma_i) (Bp - Bh_i).
Vec3 Kinematics::bodyToLegFrame(uint8_t legIndex, const Vec3 &footBodyMm) const {
    const RobotConfig::LegGeometry &geometry = RobotConfig::kLegGeometry[legIndex];
    const float dx = footBodyMm.x - geometry.coxaBodyMm.x;
    const float dy = footBodyMm.y - geometry.coxaBodyMm.y;
    const float c = cosf(-geometry.coxaMountYawRad);
    const float s = sinf(-geometry.coxaMountYawRad);

    return {
        dx * c - dy * s,
        dx * s + dy * c,
        footBodyMm.z - geometry.coxaBodyMm.z,
    };
}

// Transforms a foot position from one leg's local frame into the robot body frame
// Dimplomska enačba (4.2): Bp = Bh_i + Rz(gamma_i) Lp.
Vec3 Kinematics::legToBodyFrame(uint8_t legIndex, const Vec3 &footLegMm) const {
    const RobotConfig::LegGeometry &geometry = RobotConfig::kLegGeometry[legIndex];
    const float c = cosf(geometry.coxaMountYawRad);
    const float s = sinf(geometry.coxaMountYawRad);

    // Rotation Rz
    return {
        geometry.coxaBodyMm.x + footLegMm.x * c - footLegMm.y * s,
        geometry.coxaBodyMm.y + footLegMm.x * s + footLegMm.y * c,
        geometry.coxaBodyMm.z + footLegMm.z,
    };
}
