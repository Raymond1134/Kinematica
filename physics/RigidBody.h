#pragma once

#include "../math/Vec3.h"
#include "../math/Quat.h"
#include "Collider.h"

struct RigidBody {
    Vec3 position;
    Vec3 velocity;
    Quat orientation;
    Vec3 angularVelocity;
    float mass;
    float restitution = 0.5f;
    float friction = 0.15f;
    Collider collider;

    bool isStatic = false;

    bool sleeping = false;
    float sleepTimer = 0.0f;
    bool hadContactThisStep = false;
    
    Vec3 getInvInertiaBody() const {
        if (isStatic) return {0.0f, 0.0f, 0.0f};
        if (mass <= 0.0001f) return {0.0f, 0.0f, 0.0f};

        if (collider.type == ColliderType::Sphere) {
            float r = collider.sphere.radius;
            float I = 0.4f * mass * r * r; // 2/5 m r^2
            if (I <= 0.0001f) return {0.0f, 0.0f, 0.0f};
            float inv = 1.0f / I;
            return {inv, inv, inv};
        }

        if (collider.type == ColliderType::Box) {
            Vec3 h = collider.box.halfExtents;
            float Ixx = (1.0f / 3.0f) * mass * (h.y * h.y + h.z * h.z);
            float Iyy = (1.0f / 3.0f) * mass * (h.x * h.x + h.z * h.z);
            float Izz = (1.0f / 3.0f) * mass * (h.x * h.x + h.y * h.y);
            return {
                (Ixx > 0.0001f) ? (1.0f / Ixx) : 0.0f,
                (Iyy > 0.0001f) ? (1.0f / Iyy) : 0.0f,
                (Izz > 0.0001f) ? (1.0f / Izz) : 0.0f,
            };
        }

        float r = collider.capsule.radius;
        float L = collider.capsule.halfHeight * 2.0f;

        float Ixx = (1.0f / 12.0f) * mass * (3.0f * r * r + L * L);
        float Iyy = 0.5f * mass * r * r;
        float Izz = Ixx;

        return {
            (Ixx > 0.0001f) ? (1.0f / Ixx) : 0.0f,
            (Iyy > 0.0001f) ? (1.0f / Iyy) : 0.0f,
            (Izz > 0.0001f) ? (1.0f / Izz) : 0.0f,
        };
    }
};