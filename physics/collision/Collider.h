#pragma once

#include "../math/Vec3.h"
#include "../math/Quat.h"
#include "shapes/SphereShape.h"
#include "shapes/BoxShape.h"
#include "shapes/CapsuleShape.h"
#include "shapes/PolyhedronShape.h"
#include "shapes/ConvexShape.h"
#include "TriangleMesh.h"
#include <vector>
#include <algorithm>
#include <cassert>
#include <memory>

enum class ColliderType {Sphere, Box, Capsule, Convex, Mesh, Compound};

struct CompoundChild;
struct CompoundShape;

struct Collider : public ConvexShape {
    ColliderType type;
    SphereShape sphere;
    BoxShape box;
    CapsuleShape capsule;
    std::shared_ptr<PolyhedronShape> polyhedron;
    std::shared_ptr<TriangleMesh> mesh;
    std::shared_ptr<TriangleMesh> renderMesh;
    float meshBoundRadius = 0.0f;
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
        c.polyhedron = std::make_shared<PolyhedronShape>(verts);
        return c;
    }

    static Collider createMesh(std::shared_ptr<TriangleMesh> meshPtr, std::shared_ptr<TriangleMesh> renderMeshPtr = nullptr) {
        Collider c;
        c.type = ColliderType::Mesh;
        c.mesh = std::move(meshPtr);
        c.renderMesh = std::move(renderMeshPtr);
        c.meshBoundRadius = (c.mesh) ? c.mesh->boundRadius : 0.0f;
        return c;
    }

    static Collider createCompound(const std::vector<CompoundChild>& children);

    bool isConvex() const noexcept { return type == ColliderType::Sphere || type == ColliderType::Box || type == ColliderType::Capsule || type == ColliderType::Convex; }

    float boundingRadius() const noexcept {
        switch (type) {
            case ColliderType::Sphere:
                return sphere.radius;
            case ColliderType::Box:
                return box.halfExtents.length();
            case ColliderType::Capsule:
                return capsule.halfHeight + capsule.radius;
            case ColliderType::Convex:
                return polyhedron ? polyhedron->boundRadius : 0.0f;
            case ColliderType::Mesh:
                assert(mesh && "Mesh collider missing mesh data; use createMesh(meshPtr)");
                return (meshBoundRadius > 0.0f) ? meshBoundRadius : mesh->boundRadius;
            case ColliderType::Compound:
                return compoundBoundRadius;
        }
        return 0.0f;
    }

    Vec3 support(const Vec3& dir) const;
    void computeAabb(const Vec3& pos, const Quat& rot, Vec3& outMin, Vec3& outMax) const;
};

struct CompoundChild {
    Collider collider;
    Vec3 localPosition = {0.0f, 0.0f, 0.0f};
    Quat localOrientation = Quat::identity();
};

struct CompoundShape {
    std::vector<CompoundChild> children;
};

inline Vec3 Collider::support(const Vec3& dir) const {
    assert(type != ColliderType::Mesh && "support() called on mesh collider");

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
            assert(polyhedron && "Convex collider missing polyhedron");
            float maxDot = -1e30f;
            Vec3 best = {0, 0, 0};
            for (const Vec3& v : polyhedron->verts) {
                float dot = Vec3::dot(v, d);
                if (dot > maxDot) {
                    maxDot = dot;
                    best = v;
                }
            }
            return best;
        }
        case ColliderType::Compound: {
            if (!compound || compound->children.empty()) return {0.0f, 0.0f, 0.0f};
            float bestDot = -1e30f;
            Vec3 best = {0.0f, 0.0f, 0.0f};

            for (const CompoundChild& child : compound->children) {
                Vec3 dChild = child.localOrientation.rotateInv(d);
                Vec3 pChildLocal = child.collider.support(dChild);
                Vec3 pParentLocal = child.localPosition + child.localOrientation.rotate(pChildLocal);

                float dd = Vec3::dot(pParentLocal, d);
                if (dd > bestDot) {
                    bestDot = dd;
                    best = pParentLocal;
                }
            }

            return best;
        }
        case ColliderType::Mesh:
            break;
    }
    return {0, 0, 0};
}

inline Collider Collider::createCompound(const std::vector<CompoundChild>& children) {
    Collider c;
    c.type = ColliderType::Compound;
    c.compound = std::make_shared<CompoundShape>();

    std::vector<CompoundChild> flat;
    flat.reserve(children.size());

    auto appendFlattened = [&](auto&& self, const CompoundChild& child, const Vec3& parentPos, const Quat& parentRot, int depth) -> void {
        if (depth > 16) {
            assert(false && "Compound collider nesting too deep");
            return;
        }

        if (child.collider.type == ColliderType::Mesh) {
            assert(false && "Mesh colliders are not supported as compound children");
            return;
        }

        if (child.collider.type != ColliderType::Compound) {
            CompoundChild out = child;
            out.localPosition = parentPos + parentRot.rotate(child.localPosition);
            out.localOrientation = (parentRot * child.localOrientation).normalized();
            flat.push_back(out);
            return;
        }

        if (!child.collider.compound) {
            return;
        }

        Vec3 nextPos = parentPos + parentRot.rotate(child.localPosition);
        Quat nextRot = (parentRot * child.localOrientation).normalized();
        for (const CompoundChild& grand : child.collider.compound->children) {
            self(self, grand, nextPos, nextRot, depth + 1);
        }
    };

    for (const CompoundChild& child : children) {
        appendFlattened(appendFlattened, child, {0.0f, 0.0f, 0.0f}, Quat::identity(), 0);
    }

    c.compound->children = std::move(flat);

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