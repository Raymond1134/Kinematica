#pragma once

#include "../math/Vec3.h"

struct RigidBody {
    Vec3 position;
    Vec3 velocity;
    float mass;

    bool isStatic = false;
};