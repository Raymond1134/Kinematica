#pragma once
#include "../math/Vec3.h"
#include "../math/Quat.h"
#include "RigidBody.h"
#include <list>

struct RayHit {
    bool hit = false;
    float t = 0.0f;
    Vec3 point = {0, 0, 0};
    Vec3 normal = {0, 1, 0};
};

namespace Raycast {
    struct BodyPick {
        bool hit = false;
        float t = 0.0f;
        Vec3 point = {0, 0, 0};
        Vec3 normal = {0, 1, 0};
        RigidBody* body = nullptr;
    };

    bool raySphere(const Vec3& ro, const Vec3& rd, const Vec3& c, float r, float& outT, Vec3& outN);
    bool rayObbBox(const Vec3& ro, const Vec3& rd, const Vec3& c, const Quat& q, const Vec3& he, float& outT, Vec3& outN);
    RayHit raycastWorldPlacement(const Vec3& ro, const Vec3& rd, float floorY, const std::list<RigidBody>& bodies);
    float placementOffsetAlongNormal(const Collider& c, const Quat& q, const Vec3& nWorld);

    BodyPick pickBody(const Vec3& ro, const Vec3& rd, std::list<RigidBody>& bodies, float maxT = 250.0f);
}
