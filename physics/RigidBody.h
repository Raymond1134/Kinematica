#pragma once

#include "../math/Vec3.h"
#include "../math/Quat.h"
#include "collision/Collider.h"

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
    bool visible = true;
    float sleepTimer = 0.0f;
    float restLockTimer = 0.0f;
    bool restLocked = false;
    bool hadContactThisStep = false;
    int solverIndex = -1;
    int groupId = 0;
    
    bool useExplicitInertia = false;
    Vec3 explicitInvInertia = {0.0f, 0.0f, 0.0f};

    Vec3 getInvInertiaBody() const {
        if (isStatic) return {0.0f, 0.0f, 0.0f};
        if (mass <= 0.0001f) return {0.0f, 0.0f, 0.0f};
        
        if (useExplicitInertia) return explicitInvInertia;

        if (collider.type == ColliderType::Sphere) {
            float r = collider.sphere.radius;
            float I = 0.4f * mass * r * r; // 2/5 m r^2 constant I for now
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

        if (collider.type == ColliderType::Convex) {
            if (collider.polyhedron && !collider.polyhedron->verts.empty()) {
                Vec3 mn = collider.polyhedron->verts[0];
                Vec3 mx = collider.polyhedron->verts[0];
                for (const Vec3& v : collider.polyhedron->verts) {
                    mn.x = std::min(mn.x, v.x); mn.y = std::min(mn.y, v.y); mn.z = std::min(mn.z, v.z);
                    mx.x = std::max(mx.x, v.x); mx.y = std::max(mx.y, v.y); mx.z = std::max(mx.z, v.z);
                }
                Vec3 h = (mx - mn) * 0.5f;
                h.x = std::max(h.x, 0.001f);
                h.y = std::max(h.y, 0.001f);
                h.z = std::max(h.z, 0.001f);

                float Ixx = (1.0f / 3.0f) * mass * (h.y * h.y + h.z * h.z);
                float Iyy = (1.0f / 3.0f) * mass * (h.x * h.x + h.z * h.z);
                float Izz = (1.0f / 3.0f) * mass * (h.x * h.x + h.y * h.y);
                return {
                    (Ixx > 0.0001f) ? (1.0f / Ixx) : 0.0f,
                    (Iyy > 0.0001f) ? (1.0f / Iyy) : 0.0f,
                    (Izz > 0.0001f) ? (1.0f / Izz) : 0.0f,
                };
            }

            float r = collider.boundingRadius();
            float I = 0.4f * mass * r * r;
            if (I <= 0.0001f) return {0.0f, 0.0f, 0.0f};
            float inv = 1.0f / I;
            return {inv, inv, inv};
        }

        if (collider.type == ColliderType::Compound) {
            float r = collider.boundingRadius();
            float I = 0.4f * mass * r * r;
            if (I <= 0.0001f) return {0.0f, 0.0f, 0.0f};
            float inv = 1.0f / I;
            return {inv, inv, inv};
        }

        if (collider.type == ColliderType::Capsule) {
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

        if (collider.type == ColliderType::Mesh) {
            if (!collider.mesh) return {0.0f, 0.0f, 0.0f};
            Vec3 h = collider.mesh->localHalfExtents();
            float Ixx = (1.0f / 3.0f) * mass * (h.y * h.y + h.z * h.z);
            float Iyy = (1.0f / 3.0f) * mass * (h.x * h.x + h.z * h.z);
            float Izz = (1.0f / 3.0f) * mass * (h.x * h.x + h.y * h.y);
            return {
                (Ixx > 0.0001f) ? (1.0f / Ixx) : 0.0f,
                (Iyy > 0.0001f) ? (1.0f / Iyy) : 0.0f,
                (Izz > 0.0001f) ? (1.0f / Izz) : 0.0f,
            };
        }

        return {0.0f, 0.0f, 0.0f};
    }
};