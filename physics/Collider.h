#pragma once

#include "../math/Vec3.h"
#include "shapes/ConvexShape.h"
#include <vector>
#include <algorithm>

enum class ColliderType {
    Sphere,
    Box,
    Capsule,
    Convex
};

struct Collider : public ConvexShape {
    ColliderType type;

    struct SphereData {
        float radius = 0.5f;
    } sphere;

    struct BoxData {
        Vec3 halfExtents = {0.5f, 0.5f, 0.5f};
    } box;


    struct CapsuleData {
        float radius = 0.5f;
        float halfHeight = 0.5f;
    } capsule;

    // Convex polyhedron
    std::vector<Vec3> convexVerts;

    Collider() : type(ColliderType::Sphere) {}

    static Collider createSphere(float radius) {
        Collider c;
        c.type = ColliderType::Sphere;
        c.sphere.radius = radius;
        return c;
    }

    static Collider createBox(Vec3 halfExtents) {
        Collider c;
        c.type = ColliderType::Box;
        c.box.halfExtents = halfExtents;
        return c;
    }

    static Collider createCapsule(float radius, float halfHeight) {
        Collider c;
        c.type = ColliderType::Capsule;
        c.capsule.radius = radius;
        c.capsule.halfHeight = halfHeight;
        return c;
    }

    static Collider createConvex(const std::vector<Vec3>& verts) {
        Collider c;
        c.type = ColliderType::Convex;
        c.convexVerts = verts;
        return c;
    }

    Vec3 support(const Vec3& dir) const override {
        Vec3 d = dir.normalized();
        switch (type) {
            case ColliderType::Sphere:
                return d * sphere.radius;
            case ColliderType::Box:
                return {
                    d.x >= 0.0f ? box.halfExtents.x : -box.halfExtents.x,
                    d.y >= 0.0f ? box.halfExtents.y : -box.halfExtents.y,
                    d.z >= 0.0f ? box.halfExtents.z : -box.halfExtents.z
                };
            case ColliderType::Capsule: {
                Vec3 axis = {0.0f, 1.0f, 0.0f};
                float sign = Vec3::dot(d, axis) >= 0.0f ? 1.0f : -1.0f;
                Vec3 capCenter = axis * (capsule.halfHeight * sign);
                return capCenter + d * capsule.radius;
            }
            case ColliderType::Convex: {
                float maxDot = -1e30f;
                Vec3 best = {0,0,0};
                for (const Vec3& v : convexVerts) {
                    float dot = Vec3::dot(v, d);
                    if (dot > maxDot) {
                        maxDot = dot;
                        best = v;
                    }
                }
                return best;
            }
        }
        return {0, 0, 0};
    }
};