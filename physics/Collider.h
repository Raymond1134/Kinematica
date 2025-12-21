#pragma once

#include "../math/Vec3.h"
#include "../math/Quat.h"
#include "shapes/SphereShape.h"
#include "shapes/BoxShape.h"
#include "shapes/CapsuleShape.h"
#include "shapes/PolyhedronShape.h"
#include "shapes/ConvexShape.h"
#include <vector>
#include <algorithm>
#include <cassert>
#include <memory>

enum class ColliderType {
    Sphere,
    Box,
    Capsule,
    Convex,
    Mesh,
    Compound
};

struct CompoundChild;
struct CompoundShape;

struct Collider : public ConvexShape {
    ColliderType type;

    SphereShape sphere;
    BoxShape box;
    CapsuleShape capsule;
    PolyhedronShape polyhedron;
    std::shared_ptr<void> mesh;
    std::shared_ptr<CompoundShape> compound;
    float compoundBoundRadius = 0.0f;

    Collider() : type(ColliderType::Sphere) {}

    static Collider createSphere(float radius) {
        Collider c;
        c.type = ColliderType::Sphere;
        c.sphere = SphereShape(radius);
        return c;
    }
    static Collider createBox(Vec3 halfExtents) {
        Collider c;
        c.type = ColliderType::Box;
        c.box = BoxShape(halfExtents);
        return c;
    }
    static Collider createCapsule(float radius, float halfHeight) {
        Collider c;
        c.type = ColliderType::Capsule;
        c.capsule = CapsuleShape(radius, halfHeight);
        return c;
    }
    static Collider createConvex(const std::vector<Vec3>& verts) {
        Collider c;
        c.type = ColliderType::Convex;
        c.polyhedron = PolyhedronShape(verts);
        return c;
    }

    static Collider createCompound(const std::vector<CompoundChild>& children);

    bool isConvex() const noexcept {
        return type == ColliderType::Sphere ||
               type == ColliderType::Box ||
               type == ColliderType::Capsule ||
               type == ColliderType::Convex;
    }

    float boundingRadius() const noexcept {
        switch (type) {
            case ColliderType::Sphere:
                return sphere.radius;
            case ColliderType::Box:
                return box.halfExtents.length();
            case ColliderType::Capsule:
                return capsule.halfHeight + capsule.radius;
            case ColliderType::Convex:
                return polyhedron.boundRadius;
            case ColliderType::Mesh:
                break;
            case ColliderType::Compound:
                return compoundBoundRadius;
        }
        return 0.0f;
    }

    Vec3 support(const Vec3& dir) const override {
        assert(isConvex() && "support() called on non-convex collider");

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
                for (const Vec3& v : polyhedron.verts) {
                    float dot = Vec3::dot(v, d);
                    if (dot > maxDot) {
                        maxDot = dot;
                        best = v;
                    }
                }
                return best;
            }
            case ColliderType::Mesh:
            case ColliderType::Compound:
                break;
        }
        return {0, 0, 0};
    }
};

struct CompoundChild {
    Collider collider;
    Vec3 localPosition = {0.0f, 0.0f, 0.0f};
    Quat localOrientation = Quat::identity();
};

struct CompoundShape {
    std::vector<CompoundChild> children;
};

inline Collider Collider::createCompound(const std::vector<CompoundChild>& children) {
    Collider c;
    c.type = ColliderType::Compound;
    c.compound = std::make_shared<CompoundShape>();
    c.compound->children = children;

    float r = 0.0f;
    for (const CompoundChild& child : c.compound->children) {
        float childR = child.collider.boundingRadius();
        float d = child.localPosition.length();
        float rr = d + childR;
        if (rr > r) r = rr;
    }
    c.compoundBoundRadius = r;
    return c;
}