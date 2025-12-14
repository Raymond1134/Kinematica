#pragma once

#include "../math/Vec3.h"
#include "RigidBody.h"
#include <vector>

class PhysicsWorld {
public:
    Vec3 gravity = {0.0f, -9.81f, 0.0f};

    void addRigidBody(RigidBody* body) {
        bodies.push_back(body);
    }

    void step(float deltaTime) {
        for (RigidBody* body: bodies) {
            if (body->isStatic) continue;

            body->velocity += gravity * deltaTime;
            body->position += body->velocity * deltaTime;

            handleFloorCollision(body);
        }
    }

private:
    std::vector<RigidBody*> bodies;

    void handleFloorCollision(RigidBody* body) {
        const float floorY = 0.0f;

        if (body->position.y < floorY) {
            body->position.y = floorY;
            body->velocity.y *= -0.5f;
        }
    }
};