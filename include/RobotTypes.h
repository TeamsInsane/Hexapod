#pragma once

#include <Arduino.h>

enum class LegId : uint8_t {
    LeftFront = 0,
    LeftMiddle,
    LeftRear,
    RightFront,
    RightMiddle,
    RightRear,
};

enum class JointId : uint8_t {
    Coxa = 0,
    Femur,
    Tibia,
};

enum class ToFSensorRole : uint8_t {
    Front = 0,
    Left,
    Right,
};

struct Vec3 {
    float x;
    float y;
    float z;
};

struct FootTarget {
    Vec3 bodyPositionMm;
    bool valid;
};

// Kinematični model
struct JointAnglesDeg {
    float coxa;
    float femur;
    float tibia;
    bool valid;
};

// Raw degrees
struct ServoAnglesDeg {
    float coxa;
    float femur;
    float tibia;
    bool valid;
};

struct DistanceReading {
    uint16_t rawMm;
    uint16_t filteredMm;
    uint32_t timestampMs;
    bool valid;
    bool timedOut;
};

namespace Hexapod {
    // Horizontal
    enum class AimServoAction : uint8_t {
        Off,
        Stop,
        Left,
        Right,
    };

    // Shoot
    enum class ShootPhase : uint8_t {
        Idle,
        Forward,
        Reset,
    };

    // Vertical
    enum class TiltAction : uint8_t {
        Idle,
        Up,
        Down,
    };

    // Charge
    enum class ChargeAction : uint8_t {
        Idle,
        Charge,
        Uncharge,
    };

    // Leg state machine (what is happening right now)
    enum class StandStage : uint8_t {
        Idle,
        //Stand
        StartupPoseHold, // 90 180 170
        MovingAllTo90, // 90 90 90
        All90Pause, //pause in 90
        MovingToWalkingPose, // 90° / 150° / 140°
        WalkingPoseComplete, // Standing
        //Gait
        WaveLift, //Lift leg/pair
        WaveSwingForward, //Swing leg/pair
        WaveLower, //Lower leg/pair
        WavePush, //Push
        WaveRecoveryLift,
        WaveRecoverySwing,
        WaveRecoveryLower,
        WaveStopped,
        OutputsOff,
        Fault,
    };

    enum class StandMode : uint8_t {
        None,
        FullRobot,
    };

    // Which movement/gait is selected
    enum class WalkMode : uint8_t {
        None,
        SingleWave, //one leg with push
        DualRipple, //pair with push
        ContinuousRippleV4DistributedStance, //pair without push
        ContinuousRippleV6FullIk, //pair without push, IK
        ContinuousRippleV6ObstacleAvoidance, //pair without push, IK, obstacle avoidance
        CalculatedPath, //calculated (30cm)
        TurnClockwise,
        TurnCounterClockwise,
        TurnClockwiseFast,
        TurnCounterClockwiseFast,
    };

    enum class CalculatedPathPhase : uint8_t {
        None,
        TurnToTarget,
        MoveToTarget,
    };
} // namespace Hexapod
