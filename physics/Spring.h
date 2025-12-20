#pragma once
#include "RigidBody.h"

struct Spring {
    RigidBody* a;
    RigidBody* b;
    float restLength;
    float stiffness;
    float damping;
    Spring(RigidBody* a, RigidBody* b, float restLength, float stiffness, float damping) : a(a), b(b), restLength(restLength), stiffness(stiffness), damping(damping) {}
};
