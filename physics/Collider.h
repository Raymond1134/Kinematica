#pragma once

#include "../math/Vec3.h"

enum class ColliderType {
    Sphere,
    Box,
    Capsule
};

struct Collider {
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
};