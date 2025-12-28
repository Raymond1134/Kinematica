#include "Raycast.h"
#include "collision/TriangleMesh.h"
#include <cmath>
#include <algorithm>

namespace Raycast {

    bool raySphere(const Vec3& ro, const Vec3& rd, const Vec3& c, float r, float& outT, Vec3& outN) {
        Vec3 oc = ro - c;
        float b = Vec3::dot(oc, rd);
        float cterm = Vec3::dot(oc, oc) - r * r;
        float disc = b * b - cterm;
        if (disc < 0.0f) return false;
        float s = std::sqrt(disc);
        float t0 = -b - s;
        float t1 = -b + s;
        float t = (t0 > 1e-5f) ? t0 : ((t1 > 1e-5f) ? t1 : -1.0f);
        if (t <= 0.0f) return false;
        Vec3 p = ro + rd * t;
        outN = (p - c).normalized();
        outT = t;
        return true;
    }

    bool rayObbBox(const Vec3& ro, const Vec3& rd, const Vec3& c, const Quat& q, const Vec3& he, float& outT, Vec3& outN) {
        Vec3 roL = q.rotateInv(ro - c);
        Vec3 rdL = q.rotateInv(rd);

        float tmin = -1e30f;
        float tmax = 1e30f;
        Vec3 nEnter = {0.0f, 0.0f, 0.0f};

        auto slab = [&](float roA, float rdA, float minA, float maxA, const Vec3& nNeg, const Vec3& nPos) -> bool {
            if (std::fabs(rdA) < 1e-8f) {
                return (roA >= minA && roA <= maxA);
            }
            float inv = 1.0f / rdA;
            float t0 = (minA - roA) * inv;
            float t1 = (maxA - roA) * inv;
            Vec3 n0 = nNeg;
            Vec3 n1 = nPos;
            if (t0 > t1) { std::swap(t0, t1); std::swap(n0, n1); }
            if (t0 > tmin) { tmin = t0; nEnter = n0; }
            if (t1 < tmax) { tmax = t1; }
            return tmin <= tmax;
        };

        if (!slab(roL.x, rdL.x, -he.x, he.x, Vec3{-1, 0, 0}, Vec3{1, 0, 0})) return false;
        if (!slab(roL.y, rdL.y, -he.y, he.y, Vec3{0, -1, 0}, Vec3{0, 1, 0})) return false;
        if (!slab(roL.z, rdL.z, -he.z, he.z, Vec3{0, 0, -1}, Vec3{0, 0, 1})) return false;

        float t = (tmin > 1e-5f) ? tmin : tmax;
        if (t <= 1e-5f) return false;
        outT = t;
        outN = q.rotate(nEnter).normalized();
        return true;
    }

    static bool rayTri(const Vec3& ro, const Vec3& rd, const Vec3& a, const Vec3& b, const Vec3& c, float& outT, Vec3& outN) {
        Vec3 ab = b - a;
        Vec3 ac = c - a;
        Vec3 pvec = Vec3::cross(rd, ac);
        float det = Vec3::dot(ab, pvec);
        if (std::fabs(det) < 1e-8f) return false;
        float invDet = 1.0f / det;
        Vec3 tvec = ro - a;
        float u = Vec3::dot(tvec, pvec) * invDet;
        if (u < 0.0f || u > 1.0f) return false;
        Vec3 qvec = Vec3::cross(tvec, ab);
        float v = Vec3::dot(rd, qvec) * invDet;
        if (v < 0.0f || (u + v) > 1.0f) return false;
        float t = Vec3::dot(ac, qvec) * invDet;
        if (t <= 1e-5f) return false;

        Vec3 n = Vec3::cross(ab, ac).normalized();
        if (Vec3::dot(n, rd) > 0.0f) n = -n;
        outT = t;
        outN = n;
        return true;
    }

    static bool rayMesh(const Vec3& roW, const Vec3& rdW, const Vec3& posW, const Quat& qW, const TriangleMesh& mesh, float& outT, Vec3& outN) {
        float tS;
        Vec3 nS;
        if (!raySphere(roW, rdW, posW + qW.rotate(mesh.localCenter()), mesh.boundRadius, tS, nS)) {
            return false;
        }

        Vec3 ro = qW.rotateInv(roW - posW);
        Vec3 rd = qW.rotateInv(rdW);

        float bestT = 1e30f;
        Vec3 bestN = {0, 1, 0};

        for (const auto& t : mesh.tris) {
            if (t.a >= mesh.vertices.size() || t.b >= mesh.vertices.size() || t.c >= mesh.vertices.size()) continue;
            const Vec3& a = mesh.vertices[t.a];
            const Vec3& b = mesh.vertices[t.b];
            const Vec3& c = mesh.vertices[t.c];
            float tt;
            Vec3 nn;
            if (!rayTri(ro, rd, a, b, c, tt, nn)) continue;
            if (tt < bestT) { bestT = tt; bestN = nn; }
        }

        if (bestT >= 1e29f) return false;
        outT = bestT;
        outN = qW.rotate(bestN).normalized();
        return true;
    }

    float placementOffsetAlongNormal(const Collider& c, const Quat& q, const Vec3& nWorld) {
        Vec3 n = nWorld.normalized();
        if (c.type == ColliderType::Mesh) {
            const TriangleMesh* meshPtr = c.renderMesh ? c.renderMesh.get() : c.mesh.get();
            if (!meshPtr || meshPtr->vertices.empty()) {
                return std::max(0.01f, c.boundingRadius());
            }
            float minD = +1e30f;
            for (const Vec3& vLocal : meshPtr->vertices) {
                Vec3 vWorld = q.rotate(vLocal);
                float d = Vec3::dot(vWorld, n);
                if (d < minD) minD = d;
            }
            if (!std::isfinite(minD)) return std::max(0.01f, c.boundingRadius());
            float offset = -minD;
            if (!std::isfinite(offset)) offset = c.boundingRadius();
            return std::max(0.01f, offset);
        }
        Vec3 nLocal = q.rotateInv(n);
        Vec3 pLocal = c.support(nLocal);
        Vec3 pWorld = q.rotate(pLocal);
        float d = Vec3::dot(pWorld, n);
        if (!std::isfinite(d)) d = c.boundingRadius();
        return std::max(0.01f, d);
    }

    RayHit raycastWorldPlacement(const Vec3& ro, const Vec3& rd, float floorY, const std::list<RigidBody>& bodies) {
        RayHit best;
        best.hit = false;
        best.t = 1e30f;

        if (std::fabs(rd.y) > 1e-8f) {
            float t = (floorY - ro.y) / rd.y;
            if (t > 1e-5f && t < best.t) {
                best.hit = true;
                best.t = t;
                best.point = ro + rd * t;
                best.normal = {0.0f, 1.0f, 0.0f};
            }
        }

        for (const RigidBody& b : bodies) {
            const Collider& c = b.collider;

            float tS;
            Vec3 nS;
            float br = c.boundingRadius();
            if (br > 0.0f) {
                if (!raySphere(ro, rd, b.position, br, tS, nS)) {
                    continue;
                }
            }

            float t = 0.0f;
            Vec3 n = {0.0f, 1.0f, 0.0f};
            bool hit = false;

            switch (c.type) {
                case ColliderType::Sphere:
                    hit = raySphere(ro, rd, b.position, c.sphere.radius, t, n);
                    break;
                case ColliderType::Box:
                    hit = rayObbBox(ro, rd, b.position, b.orientation, c.box.halfExtents, t, n);
                    break;
                case ColliderType::Capsule:
                case ColliderType::Convex:
                case ColliderType::Compound:
                    hit = raySphere(ro, rd, b.position, br, t, n);
                    break;
                case ColliderType::Mesh:
                    if (c.mesh) {
                        const TriangleMesh& mesh = c.renderMesh ? *c.renderMesh : *c.mesh;
                        hit = rayMesh(ro, rd, b.position, b.orientation, mesh, t, n);
                    }
                    break;
            }

            if (hit && t > 1e-5f && t < best.t) {
                best.hit = true;
                best.t = t;
                best.point = ro + rd * t;
                best.normal = n;
            }
        }

        return best;
    }
}

Raycast::BodyPick Raycast::pickBody(const Vec3& ro, const Vec3& rd, std::list<RigidBody>& bodies, float maxT) {
    BodyPick best;
    best.hit = false;
    best.t = maxT;

    Vec3 dir = rd;
    float d2 = dir.lengthSq();
    if (d2 < 1e-12f) return best;
    if (fabsf(d2 - 1.0f) > 1e-3f) dir = dir * (1.0f / sqrtf(d2));

    for (RigidBody& b : bodies) {
        if (!b.visible) continue;

        float t = 0.0f;
        Vec3 n = {0.0f, 1.0f, 0.0f};
        bool hit = false;

        switch (b.collider.type) {
            case ColliderType::Sphere: {
                hit = raySphere(ro, dir, b.position, b.collider.sphere.radius, t, n);
                break;
            }
            case ColliderType::Box: {
                hit = rayObbBox(ro, dir, b.position, b.orientation, b.collider.box.halfExtents, t, n);
                break;
            }
            default: {
                float r = b.collider.boundingRadius();
                if (r > 0.0f) {
                    hit = raySphere(ro, dir, b.position, r, t, n);
                }
                break;
            }
        }

        if (!hit) continue;
        if (t < 0.0f || t > best.t) continue;

        best.hit = true;
        best.t = t;
        best.point = ro + dir * t;
        if (n.lengthSq() < 1e-12f) {
            Vec3 d = best.point - b.position;
            if (d.lengthSq() > 1e-12f) n = d.normalized();
            else n = {0.0f, 1.0f, 0.0f};
        }
        best.normal = n;
        best.body = &b;
    }

    return best;
}
