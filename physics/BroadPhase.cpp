#include "BroadPhase.h"
#include "collision/TriangleMesh.h"
#include "shapes/PolyhedronShape.h"
#include <cmath>
#include <algorithm>

void BroadPhase::computeAabb(const RigidBody* b, Vec3& outMin, Vec3& outMax) {
    auto absf3 = [](const Vec3& v) {
        return Vec3{std::abs(v.x), std::abs(v.y), std::abs(v.z)};
    };

    switch (b->collider.type) {
        case ColliderType::Sphere: {
            float r = b->collider.sphere.radius;
            Vec3 e{r, r, r};
            outMin = b->position - e;
            outMax = b->position + e;
            break;
        }
        case ColliderType::Box: {
            Vec3 he = b->collider.box.halfExtents;
            Vec3 ax = absf3(b->orientation.rotate({1.0f, 0.0f, 0.0f}));
            Vec3 ay = absf3(b->orientation.rotate({0.0f, 1.0f, 0.0f}));
            Vec3 az = absf3(b->orientation.rotate({0.0f, 0.0f, 1.0f}));
            Vec3 e{
                ax.x * he.x + ay.x * he.y + az.x * he.z,
                ax.y * he.x + ay.y * he.y + az.y * he.z,
                ax.z * he.x + ay.z * he.y + az.z * he.z,
            };
            outMin = b->position - e;
            outMax = b->position + e;
            break;
        }
        case ColliderType::Capsule: {
            float r = b->collider.capsule.radius;
            float hh = b->collider.capsule.halfHeight;
            Vec3 axis = absf3(b->orientation.rotate({0.0f, 1.0f, 0.0f}));
            Vec3 e{r + axis.x * hh, r + axis.y * hh, r + axis.z * hh};
            outMin = b->position - e;
            outMax = b->position + e;
            break;
        }
        case ColliderType::Convex: {
            if (b->collider.polyhedron) {
                Vec3 minV, maxV;
                {
                    Vec3 dir = {1.0f, 0.0f, 0.0f};
                    Vec3 sup = b->collider.support(b->orientation.rotateInv(dir));
                    maxV.x = (b->orientation.rotate(sup) + b->position).x;
                    sup = b->collider.support(b->orientation.rotateInv(-dir));
                    minV.x = (b->orientation.rotate(sup) + b->position).x;
                }
                {
                    Vec3 dir = {0.0f, 1.0f, 0.0f};
                    Vec3 sup = b->collider.support(b->orientation.rotateInv(dir));
                    maxV.y = (b->orientation.rotate(sup) + b->position).y;
                    sup = b->collider.support(b->orientation.rotateInv(-dir));
                    minV.y = (b->orientation.rotate(sup) + b->position).y;
                }
                {
                    Vec3 dir = {0.0f, 0.0f, 1.0f};
                    Vec3 sup = b->collider.support(b->orientation.rotateInv(dir));
                    maxV.z = (b->orientation.rotate(sup) + b->position).z;
                    sup = b->collider.support(b->orientation.rotateInv(-dir));
                    minV.z = (b->orientation.rotate(sup) + b->position).z;
                }
                outMin = minV;
                outMax = maxV;
            } else {
                float r = b->collider.boundingRadius();
                Vec3 e{r, r, r};
                outMin = b->position - e;
                outMax = b->position + e;
            }
            break;
        }
        case ColliderType::Mesh: {
            if (b->collider.mesh) {
                const TriangleMesh& mesh = b->collider.renderMesh ? *b->collider.renderMesh : *b->collider.mesh;
                Vec3 minL = mesh.localBounds.min;
                Vec3 maxL = mesh.localBounds.max;
                Vec3 centerL = (minL + maxL) * 0.5f;
                Vec3 heL = (maxL - minL) * 0.5f;
                
                Vec3 ax = absf3(b->orientation.rotate({1.0f, 0.0f, 0.0f}));
                Vec3 ay = absf3(b->orientation.rotate({0.0f, 1.0f, 0.0f}));
                Vec3 az = absf3(b->orientation.rotate({0.0f, 0.0f, 1.0f}));
                
                Vec3 e{
                    ax.x * heL.x + ay.x * heL.y + az.x * heL.z,
                    ax.y * heL.x + ay.y * heL.y + az.y * heL.z,
                    ax.z * heL.x + ay.z * heL.y + az.z * heL.z,
                };
                Vec3 centerW = b->orientation.rotate(centerL) + b->position;
                outMin = centerW - e;
                outMax = centerW + e;
            } else {
                float r = b->collider.boundingRadius();
                Vec3 e{r, r, r};
                outMin = b->position - e;
                outMax = b->position + e;
            }
            break;
        }
        case ColliderType::Compound: {
            float r = b->collider.boundingRadius();
            Vec3 e{r, r, r};
            outMin = b->position - e;
            outMax = b->position + e;
            break;
        }
    }
}

void BroadPhase::build(const std::vector<RigidBody*>& bodies) {
    constexpr float broadphaseMargin = 0.01f;

    entries.clear();
    entries.reserve(bodies.size());
    for (RigidBody* body : bodies) {
        if (!body) continue;
        Vec3 mn, mx;
        computeAabb(body, mn, mx);
        Entry e;
        e.minX = mn.x - broadphaseMargin;
        e.maxX = mx.x + broadphaseMargin;
        e.minY = mn.y - broadphaseMargin;
        e.maxY = mx.y + broadphaseMargin;
        e.minZ = mn.z - broadphaseMargin;
        e.maxZ = mx.z + broadphaseMargin;
        e.body = body;
        entries.push_back(e);
    }

    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.minX < b.minX; });

    pairs.clear();
    pairs.reserve(entries.size() * 2);

    activeList.clear();
    if (activeList.capacity() < 128) activeList.reserve(128);

    for (int i = 0; i < (int)entries.size(); ++i) {
        const Entry& cur = entries[i];
        RigidBody* bi = cur.body;
        if (!bi) continue;
        int write = 0;
        for (int k = 0; k < (int)activeList.size(); ++k) {
            if (entries[activeList[k]].maxX >= cur.minX) { activeList[write++] = activeList[k]; }
        }
        activeList.resize(write);

        for (int idx : activeList) {
            const Entry& other = entries[idx];
            RigidBody* bj = other.body;
            if (!bj) continue;
            if (bi == bj) continue;
            if (bi->isStatic && bj->isStatic) continue;
            if (bi->sleeping && bj->sleeping) continue;
            if (bi->groupId != 0 && bi->groupId == bj->groupId) continue;

            if (cur.maxY < other.minY || cur.minY > other.maxY) continue;
            if (cur.maxZ < other.minZ || cur.minZ > other.maxZ) continue;

            pairs.push_back({bi, bj});
        }

        activeList.push_back(i);
    }
}
