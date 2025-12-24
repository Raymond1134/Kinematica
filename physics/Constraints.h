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
