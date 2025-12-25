#pragma once
#include "../math/Vec3.h"
#include "../math/Mat3.h"
#include "RigidBody.h"

struct BallSocketJoint {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    Vec3 localAnchorA;
    Vec3 localAnchorB;

    Vec3 rA;
    Vec3 rB;
    Mat3 massMatrix;
    Vec3 bias;
    Vec3 impulseSum = {0.0f, 0.0f, 0.0f};
};

struct HingeJoint {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    Vec3 localAnchorA;
    Vec3 localAnchorB;
    Vec3 localAxisA;
    Vec3 localAxisB;

    bool enableMotor = false;
    float motorTargetVelocity = 0.0f;
    float maxMotorTorque = 0.0f;

    Vec3 rA;
    Vec3 rB;
    Vec3 bias;
    Mat3 massMatrix;
    Vec3 impulseSum = {0.0f, 0.0f, 0.0f};

    Vec3 b1;
    Vec3 c1;
    Vec3 t1, t2;
    float m1, m2, m3;
    Vec3 angularImpulseSum = {0.0f, 0.0f, 0.0f};

    float motorMass = 0.0f;
    float motorImpulse = 0.0f;
};
