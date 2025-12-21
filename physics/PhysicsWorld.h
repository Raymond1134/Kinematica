#pragma once

#include "../math/Vec3.h"
#include "RigidBody.h"
#include "Spring.h"

#include <vector>
#include <cmath>
#include <unordered_map>
#include <cstdint>
#include <omp.h>
#include "collision/GJK.h"
#include "collision/EPA.h"

class PhysicsWorld {
public:
    std::vector<RigidBody*> bodies;
    float currentDt = 1.0f / 60.0f;
    Vec3 gravity = {0.0f, -9.81f, 0.0f};
    std::vector<Spring> springs;

    // Ground plane at y=floorY. This is intentionally a dedicated plane contact.
    float floorY = 0.0f;
    bool enableFloor = true;
    float floorFriction = 0.5f;
    float floorRestitution = 0.05f;

    // Simple global damping. Values are in 1/seconds
    float linearDampingPerSecond = 0.04f;
    float angularDampingPerSecond = 0.04f;
    float floorContactAngularDampingPerSecond = 1.00f;

    void addRigidBody(RigidBody* body) {
        bodies.push_back(body);
    }

    void step(float deltaTime) {
        for (const Spring& spring : springs) {
            if (!spring.a || !spring.b) continue;
            Vec3 delta = spring.b->position - spring.a->position;
            float dist = delta.length();
            if (dist < 1e-6f) continue;
            Vec3 dir = delta / dist;
            float displacement = spring.restLength - dist;
            float relVel = Vec3::dot(spring.b->velocity - spring.a->velocity, dir);
            float forceMag = spring.stiffness * displacement - spring.damping * relVel;
            Vec3 force = dir * forceMag;
            if (!spring.a->isStatic && spring.a->mass > 0.0f) spring.a->velocity -= force * (deltaTime / spring.a->mass);
            if (!spring.b->isStatic && spring.b->mass > 0.0f) spring.b->velocity += force * (deltaTime / spring.b->mass);
        }
        currentDt = deltaTime;

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            RigidBody* body = bodies[i];
            if (!body) continue;
            body->hadContactThisStep = false;
        }

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            RigidBody* body = bodies[i];
            if (body->isStatic) continue;
            if (body->sleeping) continue;
            body->velocity += gravity * deltaTime;
        }

        #pragma omp parallel for schedule(static)
        for (int i = 0; i < (int)bodies.size(); ++i) {
            RigidBody* body = bodies[i];
            if (body->isStatic) continue;
            if (body->sleeping) continue;
            body->position += body->velocity * deltaTime;
            body->orientation.integrateAngularVelocity(body->angularVelocity, deltaTime);
        }

        // Clamp velocities and apply damping
        const float maxVelocity = 999.0f;
        const float maxAngularVelocity = 999.0f;
        for (RigidBody* body: bodies) {
            if (body->isStatic) continue;
            if (body->sleeping) continue;
            float speedSq = body->velocity.x * body->velocity.x + body->velocity.y * body->velocity.y + body->velocity.z * body->velocity.z;
            if (speedSq > maxVelocity * maxVelocity) {
                float speed = sqrtf(speedSq);
                body->velocity = body->velocity * (maxVelocity / speed);
            }
            float angSpeedSq = body->angularVelocity.x * body->angularVelocity.x + body->angularVelocity.y * body->angularVelocity.y + body->angularVelocity.z * body->angularVelocity.z;
            if (angSpeedSq > maxAngularVelocity * maxAngularVelocity) {
                float angSpeed = sqrtf(angSpeedSq);
                body->angularVelocity = body->angularVelocity * (maxAngularVelocity / angSpeed);
            }
            float lin = expf(-linearDampingPerSecond * deltaTime);
            float ang = expf(-angularDampingPerSecond * deltaTime);
            body->velocity = body->velocity * lin;
            body->angularVelocity = body->angularVelocity * ang;
        }

        // Sleep logic
        const float sleepLinear = 0.02f;
        const float sleepAngular = 0.01f;
        const float sleepTime = 0.4f;
        for (RigidBody* body: bodies) {
            if (body->isStatic) continue;
            if (body->sleeping) {
                body->sleepTimer = sleepTime;
                continue;
            }
            float v2 = body->velocity.x * body->velocity.x + body->velocity.y * body->velocity.y + body->velocity.z * body->velocity.z;
            float w2 = body->angularVelocity.x * body->angularVelocity.x + body->angularVelocity.y * body->angularVelocity.y + body->angularVelocity.z * body->angularVelocity.z;
            if (body->hadContactThisStep && v2 < sleepLinear * sleepLinear && w2 < sleepAngular * sleepAngular) {
                body->sleepTimer += deltaTime;
                if (body->sleepTimer >= sleepTime) {
                    body->sleeping = true;
                    body->velocity = {0.0f, 0.0f, 0.0f};
                    body->angularVelocity = {0.0f, 0.0f, 0.0f};
                }
            } else {
                body->sleepTimer = 0.0f;
            }
        }

        contacts.clear();
        buildContacts(contacts);
        computeRestitutionTargets(contacts);
        warmStartContacts(contacts);

        for (int pass = 0; pass < solverIterations; ++pass) {
            solveContacts(contacts, pass == 0);
        }

        ++frameId;
        pruneContactCache();
    }

    int solverIterations = 20;
    uint32_t frameId = 1;

    struct ContactPointState {
        Vec3 pointWorld;
        float penetration = 0.0f;
        float targetNormalVelocity = 0.0f;
        float normalImpulse = 0.0f;
        Vec3 tangentImpulse = {0.0f, 0.0f, 0.0f};
        float twistImpulse = 0.0f;
    };

    struct ContactManifold {
        RigidBody* a = nullptr;
        RigidBody* b = nullptr; // nullptr means floor plane
        Vec3 normal = {0.0f, 1.0f, 0.0f};

        float restitution = 0.0f;
        float friction = 0.0f;
        float frictionTwist = 0.0f;
        float patchR = 0.25f;

        ContactPointState points[8];
        int count = 0;
    };

    std::vector<ContactManifold> contacts;

    struct ContactKey {
        const void* a;
        const void* b;
        int32_t qx;
        int32_t qy;
        int32_t qz;

        bool operator==(const ContactKey& o) const {
            return a == o.a && b == o.b && qx == o.qx && qy == o.qy && qz == o.qz;
        }
    };

    struct ContactKeyHash {
        size_t operator()(const ContactKey& k) const noexcept {
            size_t h = 1469598103934665603ull;
            auto mix = [&](size_t v) {
                h ^= v;
                h *= 1099511628211ull;
            };
            mix(reinterpret_cast<size_t>(k.a));
            mix(reinterpret_cast<size_t>(k.b));
            mix(static_cast<size_t>(static_cast<uint32_t>(k.qx)));
            mix(static_cast<size_t>(static_cast<uint32_t>(k.qy)));
            mix(static_cast<size_t>(static_cast<uint32_t>(k.qz)));
            return h;
        }
    };

    struct CachedImpulse {
        float normalImpulse = 0.0f;
        Vec3 tangentImpulse = {0.0f, 0.0f, 0.0f};
        float twistImpulse = 0.0f;
        Vec3 normalWorld = {0.0f, 1.0f, 0.0f};
        uint32_t lastSeenFrame = 0;
    };

    std::unordered_map<ContactKey, CachedImpulse, ContactKeyHash> contactCache;

    static bool isFiniteVec3(const Vec3& v) {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    }

    static ContactKey makeContactKey(const RigidBody* a, const RigidBody* b, const Vec3& pWorld) {
        constexpr float q = 50.0f;
        auto qi = [&](float v) -> int32_t {
            if (!std::isfinite(v)) return 0;
            return (int32_t)lrintf(v * q);
        };

        if (a && b) {
            Vec3 pLocalA = a->orientation.rotateInv(pWorld - a->position);
            return {a, b, qi(pLocalA.x), qi(pLocalA.y), qi(pLocalA.z)};
        }

        return {a, b, qi(pWorld.x), qi(pWorld.y), qi(pWorld.z)};
    }

    void pruneContactCache() {
        constexpr uint32_t maxAge = 45;
        constexpr size_t maxEntries = 16384;
        for (auto it = contactCache.begin(); it != contactCache.end(); ) {
            uint32_t age = frameId - it->second.lastSeenFrame;
            if (age > maxAge) it = contactCache.erase(it);
            else ++it;
        }
        if (contactCache.size() > maxEntries) {
            contactCache.clear();
        }
    }

    void buildContacts(std::vector<ContactManifold>& out) {
        out.reserve(bodies.size() * 2);

        if (enableFloor) {
            for (RigidBody* body : bodies) {
                ContactManifold m;
                if (detectFloor(body, m)) {
                    out.push_back(m);
                }
            }
        }

        constexpr float broadphaseMargin = 0.05f;

        for (size_t i = 0; i < bodies.size(); ++i) {
            RigidBody* bi = bodies[i];
            if (!bi) continue;
            if (bi->isStatic) continue;
            float ri = bi->collider.boundingRadius();

            for (size_t j = i + 1; j < bodies.size(); ++j) {
                RigidBody* bj = bodies[j];
                if (!bj) continue;
                if (bj->isStatic && bi->isStatic) continue;
                if (bi->sleeping && bj->sleeping) continue;

                float rj = bj->collider.boundingRadius();
                Vec3 d = bj->position - bi->position;
                float maxD = ri + rj + broadphaseMargin;
                if (d.lengthSq() > maxD * maxD) continue;

                appendBodyBodyContacts(bi, bj, out);
            }
        }
    }

    bool detectFloor(RigidBody* body, ContactManifold& m) {
        if (!body) return false;
        if (body->isStatic) return false;

        m.a = body;
        m.b = nullptr;
        m.normal = {0.0f, -1.0f, 0.0f};

        m.restitution = (body->restitution + floorRestitution) * 0.5f;
        m.friction = (body->friction + floorFriction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = characteristicContactRadius(body);

        auto addPoint = [&](const Vec3& pWorldOnPlane, float pen) {
            if (m.count < 8) {
                m.points[m.count].pointWorld = pWorldOnPlane;
                m.points[m.count].penetration = pen;
                m.points[m.count].targetNormalVelocity = 0.0f;
                m.count++;
            }
        };

        std::vector<Vec3> verts;
        switch (body->collider.type) {
            case ColliderType::Sphere: {
                float r = body->collider.sphere.radius;
                verts.push_back(body->position + Vec3{0, -r, 0});
                break;
            }
            case ColliderType::Capsule: {
                float r = body->collider.capsule.radius;
                float hh = body->collider.capsule.halfHeight;
                Vec3 axis = body->orientation.rotate({0.0f, 1.0f, 0.0f});
                verts.push_back(body->position + axis * hh + Vec3{0, -r, 0});
                verts.push_back(body->position - axis * hh + Vec3{0, -r, 0});
                break;
            }
            case ColliderType::Box: {
                Vec3 he = body->collider.box.halfExtents;
                for (int dx = -1; dx <= 1; dx += 2) {
                    for (int dy = -1; dy <= 1; dy += 2) {
                        for (int dz = -1; dz <= 1; dz += 2) {
                            Vec3 local = {he.x * dx, he.y * dy, he.z * dz};
                            verts.push_back(body->orientation.rotate(local) + body->position);
                        }
                    }
                }
                break;
            }
            case ColliderType::Convex: {
                for (const Vec3& v : body->collider.polyhedron.verts) {
                    verts.push_back(body->orientation.rotate(v) + body->position);
                }
                break;
            }
            case ColliderType::Compound: {
                if (!body->collider.compound) break;
                for (const CompoundChild& child : body->collider.compound->children) {
                    Quat qChildWorld = (body->orientation * child.localOrientation).normalized();
                    Vec3 pChildWorld = body->orientation.rotate(child.localPosition) + body->position;

                    switch (child.collider.type) {
                        case ColliderType::Sphere: {
                            float r = child.collider.sphere.radius;
                            verts.push_back(pChildWorld + Vec3{0, -r, 0});
                            break;
                        }
                        case ColliderType::Capsule: {
                            float r = child.collider.capsule.radius;
                            float hh = child.collider.capsule.halfHeight;
                            Vec3 axis = qChildWorld.rotate({0.0f, 1.0f, 0.0f});
                            verts.push_back(pChildWorld + axis * hh + Vec3{0, -r, 0});
                            verts.push_back(pChildWorld - axis * hh + Vec3{0, -r, 0});
                            break;
                        }
                        case ColliderType::Box: {
                            Vec3 he = child.collider.box.halfExtents;
                            for (int dx = -1; dx <= 1; dx += 2) {
                                for (int dy = -1; dy <= 1; dy += 2) {
                                    for (int dz = -1; dz <= 1; dz += 2) {
                                        Vec3 local = {he.x * dx, he.y * dy, he.z * dz};
                                        verts.push_back(qChildWorld.rotate(local) + pChildWorld);
                                    }
                                }
                            }
                            break;
                        }
                        case ColliderType::Convex: {
                            for (const Vec3& v : child.collider.polyhedron.verts) {
                                verts.push_back(qChildWorld.rotate(v) + pChildWorld);
                            }
                            break;
                        }
                        case ColliderType::Mesh:
                        case ColliderType::Compound:
                            break;
                    }
                }
                break;
            }
            case ColliderType::Mesh:
                break;
        }
        for (const Vec3& v : verts) {
            float pen = floorY - v.y;
            if (pen > 0.0f) {
                addPoint({v.x, floorY, v.z}, pen);
            }
        }

        return m.count > 0;
    }

    void warmStartContacts(std::vector<ContactManifold>& ms) {
        constexpr float warmN = 0.90f;
        constexpr float warmT = 0.55f;
        constexpr float warmW = 0.55f;

        constexpr float minNormalAlign = 0.80f;
        constexpr float minTwistAlign = 0.95f;

        for (ContactManifold& m : ms) {
            for (int i = 0; i < m.count; ++i) {
                if (!isFiniteVec3(m.points[i].pointWorld)) continue;
                ContactKey key = makeContactKey(m.a, m.b, m.points[i].pointWorld);
                auto it = contactCache.find(key);
                if (it == contactCache.end()) continue;

                if ((m.a && m.a->sleeping) || (m.b && m.b->sleeping)) continue;

                const CachedImpulse& c = it->second;

                float nAlign = Vec3::dot(c.normalWorld, m.normal);
                if (nAlign < minNormalAlign) continue;

                float n0 = c.normalImpulse * warmN;

                Vec3 t0 = {0.0f, 0.0f, 0.0f};
                float w0 = 0.0f;

                if (m.points[i].penetration > 0.0f) {
                    t0 = c.tangentImpulse * warmT;
                    t0 = t0 - m.normal * Vec3::dot(t0, m.normal);

                    if (nAlign >= minTwistAlign) {
                        w0 = c.twistImpulse * warmW;
                    }

                    float tLenSq = t0.x * t0.x + t0.y * t0.y + t0.z * t0.z;
                    if (tLenSq > 1e-12f) {
                        float tLen = sqrtf(tLenSq);
                        float maxT = m.friction * n0;
                        if (tLen > maxT && maxT > 0.0f) {
                            t0 = t0 * (maxT / tLen);
                        }
                    }

                    float maxW = m.frictionTwist * n0 * m.patchR;
                    if (w0 > maxW) w0 = maxW;
                    if (w0 < -maxW) w0 = -maxW;
                }

                m.points[i].normalImpulse = n0;
                m.points[i].tangentImpulse = t0;
                m.points[i].twistImpulse = w0;

                Vec3 J = m.normal * n0 + t0;
                if (m.b) {
                    applyImpulseAtPoint(m.a, -J, m.points[i].pointWorld, false);
                    applyImpulseAtPoint(m.b, J, m.points[i].pointWorld, false);

                    applyAngularImpulse(m.a, m.normal * (-w0), false);
                    applyAngularImpulse(m.b, m.normal * (w0), false);
                } else {
                    applyImpulseAtPoint(m.a, -J, m.points[i].pointWorld, false);
                    applyAngularImpulse(m.a, m.normal * (-w0), false);
                }
            }
        }
    }

    static float clampf(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static void getBoxAxes(const RigidBody* b, Vec3 outAxes[3]) {
        outAxes[0] = b->orientation.rotate({1.0f, 0.0f, 0.0f});
        outAxes[1] = b->orientation.rotate({0.0f, 1.0f, 0.0f});
        outAxes[2] = b->orientation.rotate({0.0f, 0.0f, 1.0f});
    }

    static float projectBoxRadiusOnAxis(const RigidBody* b, const Vec3 axes[3], const Vec3& axisUnit) {
        const Vec3 he = b->collider.box.halfExtents;
        float r = 0.0f;
        r += he.x * fabsf(Vec3::dot(axisUnit, axes[0]));
        r += he.y * fabsf(Vec3::dot(axisUnit, axes[1]));
        r += he.z * fabsf(Vec3::dot(axisUnit, axes[2]));
        return r;
    }

    static int clipPolygonToPlane(const Vec3* inVerts, int inCount, Vec3* outVerts, const Vec3& n, float d) {
        if (inCount <= 0) return 0;
        int outCount = 0;
        Vec3 prev = inVerts[inCount - 1];
        float prevDist = Vec3::dot(n, prev) - d;

        for (int i = 0; i < inCount; ++i) {
            Vec3 cur = inVerts[i];
            float curDist = Vec3::dot(n, cur) - d;

            const bool curIn = (curDist <= 0.0f);
            const bool prevIn = (prevDist <= 0.0f);

            if (curIn ^ prevIn) {
                float t = prevDist / (prevDist - curDist);
                Vec3 hit = prev + (cur - prev) * t;
                outVerts[outCount++] = hit;
            }
            if (curIn) {
                outVerts[outCount++] = cur;
            }

            prev = cur;
            prevDist = curDist;
        }
        return outCount;
    }

    bool collideSphereSphere(RigidBody* a, RigidBody* b, ContactManifold& m) {
        float ra = a->collider.sphere.radius;
        float rb = b->collider.sphere.radius;
        Vec3 d = b->position - a->position;
        float distSq = d.lengthSq();
        float r = ra + rb;
        if (distSq > r * r) return false;

        float dist = sqrtf(distSq);
        Vec3 n = (dist > 1e-6f) ? (d / dist) : Vec3{0.0f, 1.0f, 0.0f};
        float pen = r - dist;
        if (pen < 1e-5f) pen = 1e-5f;

        Vec3 pA = a->position + n * ra;
        Vec3 pB = b->position - n * rb;
        Vec3 p = (pA + pB) * 0.5f;

        m.a = a;
        m.b = b;
        m.normal = n;
        m.restitution = (a->restitution + b->restitution) * 0.5f;
        m.friction = (a->friction + b->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));

        m.count = 1;
        m.points[0].pointWorld = p;
        m.points[0].penetration = pen;
        m.points[0].targetNormalVelocity = 0.0f;
        return true;
    }

    bool collideSphereBox(RigidBody* sphereBody, RigidBody* boxBody, ContactManifold& m) {
        const float r = sphereBody->collider.sphere.radius;
        const Vec3 he = boxBody->collider.box.halfExtents;

        Vec3 cLocal = boxBody->orientation.rotateInv(sphereBody->position - boxBody->position);

        bool inside = (fabsf(cLocal.x) <= he.x && fabsf(cLocal.y) <= he.y && fabsf(cLocal.z) <= he.z);

        Vec3 closestLocal;
        float penetration = 0.0f;

        if (!inside) {
            closestLocal = {
                clampf(cLocal.x, -he.x, he.x),
                clampf(cLocal.y, -he.y, he.y),
                clampf(cLocal.z, -he.z, he.z)
            };

            Vec3 closestWorld = boxBody->orientation.rotate(closestLocal) + boxBody->position;
            Vec3 d = closestWorld - sphereBody->position;
            float distSq = d.lengthSq();
            if (distSq > r * r) return false;
            float dist = sqrtf(distSq);
            Vec3 n = (dist > 1e-6f) ? (d / dist) : Vec3{0.0f, 1.0f, 0.0f}; // sphere -> box
            penetration = r - dist;
            if (penetration < 1e-5f) penetration = 1e-5f;

            Vec3 pA = sphereBody->position + n * r;
            Vec3 pB = closestWorld;
            Vec3 p = (pA + pB) * 0.5f;

            m.a = sphereBody;
            m.b = boxBody;
            m.normal = n;
            m.restitution = (sphereBody->restitution + boxBody->restitution) * 0.5f;
            m.friction = (sphereBody->friction + boxBody->friction) * 0.5f;
            m.frictionTwist = m.friction * 0.35f;
            m.patchR = 0.5f * (characteristicContactRadius(sphereBody) + characteristicContactRadius(boxBody));

            m.count = 1;
            m.points[0].pointWorld = p;
            m.points[0].penetration = penetration;
            m.points[0].targetNormalVelocity = 0.0f;
            return true;
        }

        float dx = he.x - fabsf(cLocal.x);
        float dy = he.y - fabsf(cLocal.y);
        float dz = he.z - fabsf(cLocal.z);

        Vec3 nLocal = {1.0f, 0.0f, 0.0f};
        float faceDist = dx;
        if (dy < faceDist) { faceDist = dy; nLocal = {0.0f, 1.0f, 0.0f}; }
        if (dz < faceDist) { faceDist = dz; nLocal = {0.0f, 0.0f, 1.0f}; }

        float sx = (cLocal.x >= 0.0f) ? 1.0f : -1.0f;
        float sy = (cLocal.y >= 0.0f) ? 1.0f : -1.0f;
        float sz = (cLocal.z >= 0.0f) ? 1.0f : -1.0f;
        if (nLocal.x != 0.0f) nLocal.x *= sx;
        if (nLocal.y != 0.0f) nLocal.y *= sy;
        if (nLocal.z != 0.0f) nLocal.z *= sz;

        closestLocal = cLocal + nLocal * faceDist;
        Vec3 closestWorld = boxBody->orientation.rotate(closestLocal) + boxBody->position;
        Vec3 n = boxBody->orientation.rotate(nLocal);
        penetration = r + faceDist;
        if (penetration < 1e-5f) penetration = 1e-5f;

        Vec3 pA = sphereBody->position + n * r;
        Vec3 pB = closestWorld;
        Vec3 p = (pA + pB) * 0.5f;

        m.a = sphereBody;
        m.b = boxBody;
        m.normal = n;
        m.restitution = (sphereBody->restitution + boxBody->restitution) * 0.5f;
        m.friction = (sphereBody->friction + boxBody->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(sphereBody) + characteristicContactRadius(boxBody));

        m.count = 1;
        m.points[0].pointWorld = p;
        m.points[0].penetration = penetration;
        m.points[0].targetNormalVelocity = 0.0f;
        return true;
    }

    static float closestPtSegmentSegment(
        const Vec3& p1,
        const Vec3& q1,
        const Vec3& p2,
        const Vec3& q2,
        float& s,
        float& t,
        Vec3& c1,
        Vec3& c2
    ) {
        Vec3 d1 = q1 - p1;
        Vec3 d2 = q2 - p2;
        Vec3 r = p1 - p2;
        float a = Vec3::dot(d1, d1);
        float e = Vec3::dot(d2, d2);
        float f = Vec3::dot(d2, r);

        const float eps = 1e-8f;
        if (a <= eps && e <= eps) {
            s = 0.0f;
            t = 0.0f;
            c1 = p1;
            c2 = p2;
            return (c1 - c2).lengthSq();
        }
        if (a <= eps) {
            s = 0.0f;
            t = clampf(f / e, 0.0f, 1.0f);
        } else {
            float c = Vec3::dot(d1, r);
            if (e <= eps) {
                t = 0.0f;
                s = clampf(-c / a, 0.0f, 1.0f);
            } else {
                float b = Vec3::dot(d1, d2);
                float denom = a * e - b * b;
                if (denom != 0.0f) {
                    s = clampf((b * f - c * e) / denom, 0.0f, 1.0f);
                } else {
                    s = 0.0f;
                }
                t = (b * s + f) / e;
                if (t < 0.0f) {
                    t = 0.0f;
                    s = clampf(-c / a, 0.0f, 1.0f);
                } else if (t > 1.0f) {
                    t = 1.0f;
                    s = clampf((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
        return (c1 - c2).lengthSq();
    }

    bool collideCapsuleSphere(RigidBody* capsuleBody, RigidBody* sphereBody, ContactManifold& m) {
        const float rc = capsuleBody->collider.capsule.radius;
        const float hh = capsuleBody->collider.capsule.halfHeight;
        const float rs = sphereBody->collider.sphere.radius;

        Vec3 axis = capsuleBody->orientation.rotate({0.0f, 1.0f, 0.0f});
        Vec3 p0 = capsuleBody->position + axis * hh;
        Vec3 p1 = capsuleBody->position - axis * hh;

        Vec3 seg = p1 - p0;
        float segLenSq = seg.lengthSq();
        float u = 0.0f;
        if (segLenSq > 1e-12f) {
            u = Vec3::dot(sphereBody->position - p0, seg) / segLenSq;
            u = clampf(u, 0.0f, 1.0f);
        }
        Vec3 cCapsule = p0 + seg * u;
        Vec3 d = sphereBody->position - cCapsule;
        float distSq = d.lengthSq();
        float r = rc + rs;
        if (distSq > r * r) return false;

        float dist = sqrtf(distSq);
        Vec3 n = (dist > 1e-6f) ? (d / dist) : Vec3{0.0f, 1.0f, 0.0f};
        float pen = r - dist;
        if (pen < 1e-5f) pen = 1e-5f;

        Vec3 pA = cCapsule + n * rc;
        Vec3 pB = sphereBody->position - n * rs;
        Vec3 p = (pA + pB) * 0.5f;

        m.a = capsuleBody;
        m.b = sphereBody;
        m.normal = n;
        m.restitution = (capsuleBody->restitution + sphereBody->restitution) * 0.5f;
        m.friction = (capsuleBody->friction + sphereBody->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(capsuleBody) + characteristicContactRadius(sphereBody));

        m.count = 1;
        m.points[0].pointWorld = p;
        m.points[0].penetration = pen;
        m.points[0].targetNormalVelocity = 0.0f;
        return true;
    }

    bool collideCapsuleCapsule(RigidBody* a, RigidBody* b, ContactManifold& m) {
        const float ra = a->collider.capsule.radius;
        const float rha = a->collider.capsule.halfHeight;
        const float rb = b->collider.capsule.radius;
        const float rhb = b->collider.capsule.halfHeight;

        Vec3 axisA = a->orientation.rotate({0.0f, 1.0f, 0.0f});
        Vec3 axisB = b->orientation.rotate({0.0f, 1.0f, 0.0f});

        Vec3 a0 = a->position + axisA * rha;
        Vec3 a1 = a->position - axisA * rha;
        Vec3 b0 = b->position + axisB * rhb;
        Vec3 b1 = b->position - axisB * rhb;

        float s = 0.0f, t = 0.0f;
        Vec3 c1, c2;
        float distSq = closestPtSegmentSegment(a0, a1, b0, b1, s, t, c1, c2);
        float r = ra + rb;
        if (distSq > r * r) return false;

        float dist = sqrtf(distSq);
        Vec3 d = c2 - c1;
        Vec3 n = (dist > 1e-6f) ? (d / dist) : Vec3{0.0f, 1.0f, 0.0f};
        float pen = r - dist;
        if (pen < 1e-5f) pen = 1e-5f;

        Vec3 pA = c1 + n * ra;
        Vec3 pB = c2 - n * rb;
        Vec3 p = (pA + pB) * 0.5f;

        m.a = a;
        m.b = b;
        m.normal = n;
        m.restitution = (a->restitution + b->restitution) * 0.5f;
        m.friction = (a->friction + b->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));

        m.count = 1;
        m.points[0].pointWorld = p;
        m.points[0].penetration = pen;
        m.points[0].targetNormalVelocity = 0.0f;
        return true;
    }

    bool collideBoxBox(RigidBody* a, RigidBody* b, ContactManifold& m) {
        const Vec3 heA = a->collider.box.halfExtents;
        const Vec3 heB = b->collider.box.halfExtents;

        Vec3 Ax[3], Bx[3];
        getBoxAxes(a, Ax);
        getBoxAxes(b, Bx);

        Vec3 d = b->position - a->position;

        float bestOverlap = std::numeric_limits<float>::infinity();
        Vec3 bestAxis = {0.0f, 1.0f, 0.0f};
        int bestAxisType = -1;
        int bestAxisIndex = -1;

        auto testAxis = [&](const Vec3& axis, int axisType, int axisIndex) -> bool {
            float lenSq = axis.lengthSq();
            if (lenSq < 1e-12f) return true;
            Vec3 n = axis / sqrtf(lenSq);
            float ra = heA.x * fabsf(Vec3::dot(n, Ax[0])) +
                       heA.y * fabsf(Vec3::dot(n, Ax[1])) +
                       heA.z * fabsf(Vec3::dot(n, Ax[2]));
            float rb = heB.x * fabsf(Vec3::dot(n, Bx[0])) +
                       heB.y * fabsf(Vec3::dot(n, Bx[1])) +
                       heB.z * fabsf(Vec3::dot(n, Bx[2]));
            float dist = fabsf(Vec3::dot(d, n));
            float overlap = (ra + rb) - dist;
            if (overlap < 0.0f) return false;
            if (overlap < bestOverlap) {
                bestOverlap = overlap;
                bestAxis = n;
                bestAxisType = axisType;
                bestAxisIndex = axisIndex;
            }
            return true;
        };

        for (int i = 0; i < 3; ++i) {
            if (!testAxis(Ax[i], 0, i)) return false;
        }
        for (int i = 0; i < 3; ++i) {
            if (!testAxis(Bx[i], 1, i)) return false;
        }

        int crossIndex = 0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Vec3 c = Vec3::cross(Ax[i], Bx[j]);
                if (!testAxis(c, 2, crossIndex)) return false;
                ++crossIndex;
            }
        }

        if (Vec3::dot(bestAxis, d) < 0.0f) {
            bestAxis = -bestAxis;
        }
        if (bestOverlap < 1e-5f) bestOverlap = 1e-5f;

        if (bestAxisType == 2) {
            return false;
        }

        RigidBody* ref = (bestAxisType == 0) ? a : b;
        RigidBody* inc = (bestAxisType == 0) ? b : a;
        Vec3 n = (bestAxisType == 0) ? bestAxis : -bestAxis;

        Vec3 refAxes[3], incAxes[3];
        getBoxAxes(ref, refAxes);
        getBoxAxes(inc, incAxes);
        Vec3 refHe = ref->collider.box.halfExtents;
        Vec3 incHe = inc->collider.box.halfExtents;

        int refAxis = bestAxisIndex;
        float refSign = (Vec3::dot(n, refAxes[refAxis]) >= 0.0f) ? 1.0f : -1.0f;
        Vec3 refNormal = refAxes[refAxis] * refSign;
        Vec3 refCenter = ref->position + refNormal * ((refAxis == 0) ? refHe.x : (refAxis == 1 ? refHe.y : refHe.z));

        int uAxis = (refAxis + 1) % 3;
        int vAxis = (refAxis + 2) % 3;
        Vec3 u = refAxes[uAxis];
        Vec3 v = refAxes[vAxis];
        float hu = (uAxis == 0) ? refHe.x : (uAxis == 1 ? refHe.y : refHe.z);
        float hv = (vAxis == 0) ? refHe.x : (vAxis == 1 ? refHe.y : refHe.z);

        int incFaceAxis = 0;
        float minDot = Vec3::dot(incAxes[0], refNormal);
        for (int i = 1; i < 3; ++i) {
            float d0 = Vec3::dot(incAxes[i], refNormal);
            if (d0 < minDot) {
                minDot = d0;
                incFaceAxis = i;
            }
        }
        float incSign = (Vec3::dot(incAxes[incFaceAxis], refNormal) > 0.0f) ? -1.0f : 1.0f;
        Vec3 incNormal = incAxes[incFaceAxis] * incSign;
        float incHn = (incFaceAxis == 0) ? incHe.x : (incFaceAxis == 1 ? incHe.y : incHe.z);
        Vec3 incCenter = inc->position + incNormal * incHn;

        int incUAxis = (incFaceAxis + 1) % 3;
        int incVAxis = (incFaceAxis + 2) % 3;
        Vec3 iu = incAxes[incUAxis];
        Vec3 iv = incAxes[incVAxis];
        float ihu = (incUAxis == 0) ? incHe.x : (incUAxis == 1 ? incHe.y : incHe.z);
        float ihv = (incVAxis == 0) ? incHe.x : (incVAxis == 1 ? incHe.y : incHe.z);

        Vec3 incPoly[8];
        int incCount = 4;
        incPoly[0] = incCenter + iu * ihu + iv * ihv;
        incPoly[1] = incCenter - iu * ihu + iv * ihv;
        incPoly[2] = incCenter - iu * ihu - iv * ihv;
        incPoly[3] = incCenter + iu * ihu - iv * ihv;

        Vec3 tmp1[16];
        Vec3 tmp2[16];
        int count = incCount;
        for (int i = 0; i < count; ++i) tmp1[i] = incPoly[i];

        count = clipPolygonToPlane(tmp1, count, tmp2, u, Vec3::dot(u, refCenter) + hu);
        if (count == 0) return false;
        count = clipPolygonToPlane(tmp2, count, tmp1, -u, Vec3::dot(-u, refCenter) + hu);
        if (count == 0) return false;

        count = clipPolygonToPlane(tmp1, count, tmp2, v, Vec3::dot(v, refCenter) + hv);
        if (count == 0) return false;
        count = clipPolygonToPlane(tmp2, count, tmp1, -v, Vec3::dot(-v, refCenter) + hv);
        if (count == 0) return false;

        float planeD = Vec3::dot(refNormal, refCenter);
        m.count = 0;
        for (int i = 0; i < count && m.count < 4; ++i) {
            Vec3 p = tmp1[i];
            float distToPlane = Vec3::dot(refNormal, p) - planeD;
            float pen = -distToPlane;
            if (pen <= 0.0f) continue;
            Vec3 projected = p - refNormal * distToPlane;
            m.points[m.count].pointWorld = projected;
            m.points[m.count].penetration = pen;
            m.points[m.count].targetNormalVelocity = 0.0f;
            m.count++;
        }
        if (m.count == 0) {
            Vec3 p = (a->position + b->position) * 0.5f;
            m.count = 1;
            m.points[0].pointWorld = p;
            m.points[0].penetration = bestOverlap;
            m.points[0].targetNormalVelocity = 0.0f;
        }

        if (ref == a) {
            m.a = a;
            m.b = b;
            m.normal = bestAxis;
        } else {
            m.a = a;
            m.b = b;
            m.normal = bestAxis;
        }
        m.restitution = (a->restitution + b->restitution) * 0.5f;
        m.friction = (a->friction + b->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));
        return true;
    }

    void computeRestitutionTargets(std::vector<ContactManifold>& ms) {
        constexpr float restitutionVelThreshold = 0.05f;
        for (ContactManifold& m : ms) {
            if (!m.a) continue;
            for (int i = 0; i < m.count; ++i) {
                ContactPointState& cp = m.points[i];
                Vec3 p = cp.pointWorld;

                Vec3 rA = p - m.a->position;
                Vec3 vA = m.a->velocity + Vec3::cross(m.a->angularVelocity, rA);

                Vec3 vB = {0.0f, 0.0f, 0.0f};
                if (m.b) {
                    Vec3 rB = p - m.b->position;
                    vB = m.b->velocity + Vec3::cross(m.b->angularVelocity, rB);
                }

                Vec3 relV = vB - vA;
                float vn = Vec3::dot(relV, m.normal);
                cp.targetNormalVelocity = (vn < -restitutionVelThreshold) ? (-m.restitution * vn) : 0.0f;
            }
        }
    }

    void solveContacts(std::vector<ContactManifold>& ms, bool applyPositionCorrection) {
        constexpr float slop = 0.005f;
        constexpr float maxPositionCorrection = 0.08f;

        // Penetration bias (Baumgarte-style)
        constexpr float baumgarte = 0.35f;
        constexpr float maxBias = 3.0f;

        constexpr float shockLowerInvScale = 0.25f;
        constexpr float shockNormalY = 0.55f;

        for (ContactManifold& m : ms) {
            if (!m.a) continue;
            if (m.a->isStatic) continue;
            if (m.a->sleeping) continue;

            float invMassA = (!m.a->isStatic && m.a->mass > 0.0001f) ? (1.0f / m.a->mass) : 0.0f;
            float invMassB = (m.b && !m.b->isStatic && !m.b->sleeping && m.b->mass > 0.0001f) ? (1.0f / m.b->mass) : 0.0f;

            float invScaleA = 1.0f;
            float invScaleB = 1.0f;
            if (m.b && !m.a->isStatic && !m.b->isStatic && fabsf(m.normal.y) >= shockNormalY) {
                if (m.a->position.y < m.b->position.y) invScaleA = shockLowerInvScale;
                else invScaleB = shockLowerInvScale;
            }

            invMassA *= invScaleA;
            invMassB *= invScaleB;
            float totalInvMass = invMassA + invMassB;
            if (totalInvMass < 1e-8f) continue;

            auto invInertiaWorldMulScaled = [&](const RigidBody* body, const Vec3& v, float invScale) -> Vec3 {
                if (!body) return {0.0f, 0.0f, 0.0f};
                if (body->isStatic) return {0.0f, 0.0f, 0.0f};
                if (body->sleeping) return {0.0f, 0.0f, 0.0f};
                return invInertiaWorldMul(body, v) * invScale;
            };

            m.a->hadContactThisStep = true;
            m.a->sleeping = false;
            m.a->sleepTimer = 0.0f;
            if (m.b) {
                m.b->hadContactThisStep = true;
                if (!m.b->sleeping) {
                    m.b->sleepTimer = 0.0f;
                }
            }

            if (applyPositionCorrection) {
                float maxPen = 0.0f;
                for (int i = 0; i < m.count; ++i) if (m.points[i].penetration > maxPen) maxPen = m.points[i].penetration;

                float corrMag = (maxPen - slop > 0.0f) ? (maxPen - slop) : 0.0f;
                if (corrMag > maxPositionCorrection) corrMag = maxPositionCorrection;

                if (corrMag > 0.0f) {
                    float corrRatio = (m.b == nullptr || m.b->isStatic) ? 0.80f : 0.65f;
                    Vec3 corr = m.normal * (corrMag * corrRatio);
                    if (m.b) {
                        if (!m.a->isStatic) m.a->position = m.a->position - corr * (invMassA / totalInvMass);
                        if (!m.b->isStatic) m.b->position = m.b->position + corr * (invMassB / totalInvMass);
                    } else {
                        if (!m.a->isStatic) m.a->position = m.a->position - corr;
                    }
                }
            }

            float invCount = 1.0f / (float)m.count;
            float supportImpulse = 0.0f;
            if (totalInvMass > 1e-6f) {
                supportImpulse = (fabsf(gravity.y) * currentDt / totalInvMass) * invCount;
            }

            for (int ci = 0; ci < m.count; ++ci) {
                ContactPointState& cp = m.points[ci];
                if (!isFiniteVec3(cp.pointWorld)) continue;
                Vec3 p = cp.pointWorld;

                Vec3 rA = p - m.a->position;
                Vec3 rB = (m.b) ? (p - m.b->position) : Vec3{0.0f, 0.0f, 0.0f};

                Vec3 vA = m.a->velocity + Vec3::cross(m.a->angularVelocity, rA);
                Vec3 vB = (m.b) ? (m.b->velocity + Vec3::cross(m.b->angularVelocity, rB)) : Vec3{0.0f, 0.0f, 0.0f};
                Vec3 relV = vB - vA;

                float vn = Vec3::dot(relV, m.normal);

                Vec3 rAxN = Vec3::cross(rA, m.normal);
                float denomN = invMassA + Vec3::dot(m.normal, Vec3::cross(invInertiaWorldMulScaled(m.a, rAxN, invScaleA), rA));
                if (m.b) {
                    Vec3 rBxN = Vec3::cross(rB, m.normal);
                    denomN += invMassB + Vec3::dot(m.normal, Vec3::cross(invInertiaWorldMulScaled(m.b, rBxN, invScaleB), rB));
                }
                if (denomN < 1e-6f) continue;

                float bias = 0.0f;
                if (cp.penetration > slop && currentDt > 1e-6f) {
                    bias = baumgarte * (cp.penetration - slop) / currentDt;
                    if (bias > maxBias) bias = maxBias;
                }

                float desired = -(vn - (cp.targetNormalVelocity + bias)) / denomN;

                float oldN = cp.normalImpulse;
                float newN = oldN + desired;
                if (newN < 0.0f) newN = 0.0f;
                float dN = newN - oldN;
                cp.normalImpulse = newN;

                Vec3 Jn = m.normal * dN;
                if (m.b) {
                    applyImpulseAtPoint(m.a, -Jn, p);
                    applyImpulseAtPoint(m.b, Jn, p);
                } else {
                    applyImpulseAtPoint(m.a, -Jn, p);
                }

                vA = m.a->velocity + Vec3::cross(m.a->angularVelocity, rA);
                vB = (m.b) ? (m.b->velocity + Vec3::cross(m.b->angularVelocity, rB)) : Vec3{0.0f, 0.0f, 0.0f};
                relV = vB - vA;

                Vec3 tangent = relV - m.normal * Vec3::dot(relV, m.normal);
                float tLenSq = tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z;
                if (tLenSq > 1e-8f) {
                    float tLen = sqrtf(tLenSq);
                    tangent = tangent * (1.0f / tLen);

                    Vec3 rAxT = Vec3::cross(rA, tangent);
                    float denomT = invMassA + Vec3::dot(tangent, Vec3::cross(invInertiaWorldMulScaled(m.a, rAxT, invScaleA), rA));
                    if (m.b) {
                        Vec3 rBxT = Vec3::cross(rB, tangent);
                        denomT += invMassB + Vec3::dot(tangent, Vec3::cross(invInertiaWorldMulScaled(m.b, rBxT, invScaleB), rB));
                    }
                    if (denomT > 1e-6f) {
                        float vt = Vec3::dot(relV, tangent);
                        float dT = (-vt / denomT);
                        float oldT = Vec3::dot(cp.tangentImpulse, tangent);
                        float newT = oldT + dT;
                        float nRef = (cp.normalImpulse > supportImpulse) ? cp.normalImpulse : supportImpulse;
                        float maxF = m.friction * nRef;
                        if (newT > maxF) newT = maxF;
                        if (newT < -maxF) newT = -maxF;
                        float dAcc = newT - oldT;
                        cp.tangentImpulse = tangent * newT;

                        Vec3 Jt = tangent * dAcc;
                        if (m.b) {
                            applyImpulseAtPoint(m.a, -Jt, p);
                            applyImpulseAtPoint(m.b, Jt, p);
                        } else {
                            applyImpulseAtPoint(m.a, -Jt, p);
                        }
                    }
                }

                float wRelN = 0.0f;
                if (m.b) wRelN = Vec3::dot((m.b->angularVelocity - m.a->angularVelocity), m.normal);
                else wRelN = -Vec3::dot(m.a->angularVelocity, m.normal);

                if (fabsf(wRelN) > 1e-6f) {
                    float denomTw = Vec3::dot(m.normal, invInertiaWorldMulScaled(m.a, m.normal, invScaleA));
                    if (m.b) denomTw += Vec3::dot(m.normal, invInertiaWorldMulScaled(m.b, m.normal, invScaleB));
                    if (denomTw > 1e-6f) {
                        float dW = (-wRelN / denomTw);
                        float oldW = cp.twistImpulse;
                        float newW = oldW + dW;
                        float nRef = (cp.normalImpulse > supportImpulse) ? cp.normalImpulse : supportImpulse;
                        float maxTw = m.frictionTwist * nRef * m.patchR;
                        if (newW > maxTw) newW = maxTw;
                        if (newW < -maxTw) newW = -maxTw;
                        float dTw = newW - oldW;
                        cp.twistImpulse = newW;

                        if (m.b) {
                            applyAngularImpulse(m.a, m.normal * (-dTw));
                            applyAngularImpulse(m.b, m.normal * (dTw));
                        } else {
                            applyAngularImpulse(m.a, m.normal * (-dTw));
                        }
                    }
                }
            }

            if (applyPositionCorrection && m.b == nullptr &&
                (m.a->collider.type == ColliderType::Sphere || m.a->collider.type == ColliderType::Capsule)) {
                float sumN = 0.0f;
                for (int i = 0; i < m.count; ++i) sumN += m.points[i].normalImpulse;
                if (sumN > 1e-4f) {
                    float ang = expf(-floorContactAngularDampingPerSecond * currentDt);
                    m.a->angularVelocity = m.a->angularVelocity * ang;
                }
            }

            for (int i = 0; i < m.count; ++i) {
                if (!isFiniteVec3(m.points[i].pointWorld)) continue;
                ContactKey key = makeContactKey(m.a, m.b, m.points[i].pointWorld);
                CachedImpulse& c = contactCache[key];
                c.normalImpulse = m.points[i].normalImpulse;
                c.tangentImpulse = m.points[i].tangentImpulse;
                c.twistImpulse = m.points[i].twistImpulse;
                c.normalWorld = m.normal;
                c.lastSeenFrame = frameId;
            }
        }
    }

    static RigidBody makeChildProxyBody(const RigidBody* parent, const CompoundChild& child) {
        RigidBody proxy;
        proxy.position = parent->orientation.rotate(child.localPosition) + parent->position;
        proxy.orientation = (parent->orientation * child.localOrientation).normalized();
        proxy.velocity = parent->velocity;
        proxy.angularVelocity = parent->angularVelocity;
        proxy.mass = parent->mass;
        proxy.restitution = parent->restitution;
        proxy.friction = parent->friction;
        proxy.collider = child.collider;
        proxy.isStatic = parent->isStatic;
        proxy.sleeping = false;
        return proxy;
    }

    static void orientNormalForPair(ContactManifold& m, const RigidBody* a, const RigidBody* b) {
        Vec3 ab = b->position - a->position;
        if (Vec3::dot(m.normal, ab) < 0.0f) m.normal = -m.normal;
    }

    void appendBodyBodyContacts(RigidBody* a, RigidBody* b, std::vector<ContactManifold>& out) {
        if (!a || !b) return;
        if (a->isStatic && b->isStatic) return;
        if (a->sleeping && b->sleeping) return;

        if (a->sleeping && !b->sleeping) std::swap(a, b);

        constexpr int maxManifoldsPerPair = 4;
        int emitted = 0;

        constexpr float childBroadphaseMargin = 0.02f;

        bool aCompound = (a->collider.type == ColliderType::Compound);
        bool bCompound = (b->collider.type == ColliderType::Compound);

        if (!aCompound && !bCompound) {
            ContactManifold m;
            if (detectBodyBodyConvex(a, b, m)) out.push_back(m);
            return;
        }

        if (aCompound && (!a->collider.compound)) return;
        if (bCompound && (!b->collider.compound)) return;

        if (aCompound && bCompound) {
            for (const CompoundChild& ca : a->collider.compound->children) {
                RigidBody pa = makeChildProxyBody(a, ca);
                float ra = ca.collider.boundingRadius();
                for (const CompoundChild& cb : b->collider.compound->children) {
                    RigidBody pb = makeChildProxyBody(b, cb);
                    float rb = cb.collider.boundingRadius();
                    Vec3 d = pb.position - pa.position;
                    float maxD = ra + rb + childBroadphaseMargin;
                    if (d.lengthSq() > maxD * maxD) continue;

                    ContactManifold mChild;
                    if (detectBodyBodyConvex(&pa, &pb, mChild)) {
                        mChild.a = a;
                        mChild.b = b;
                        orientNormalForPair(mChild, a, b);
                        out.push_back(mChild);
                        if (++emitted >= maxManifoldsPerPair) return;
                    }
                }
            }
            return;
        }

        if (aCompound) {
            for (const CompoundChild& ca : a->collider.compound->children) {
                RigidBody pa = makeChildProxyBody(a, ca);
                float ra = ca.collider.boundingRadius();
                float rb = b->collider.boundingRadius();
                Vec3 d = b->position - pa.position;
                float maxD = ra + rb + childBroadphaseMargin;
                if (d.lengthSq() > maxD * maxD) continue;

                ContactManifold mChild;
                if (detectBodyBodyConvex(&pa, b, mChild)) {
                    mChild.a = a;
                    mChild.b = b;
                    orientNormalForPair(mChild, a, b);
                    out.push_back(mChild);
                    if (++emitted >= maxManifoldsPerPair) return;
                }
            }
            return;
        }

        for (const CompoundChild& cb : b->collider.compound->children) {
            RigidBody pb = makeChildProxyBody(b, cb);
            float ra = a->collider.boundingRadius();
            float rb = cb.collider.boundingRadius();
            Vec3 d = pb.position - a->position;
            float maxD = ra + rb + childBroadphaseMargin;
            if (d.lengthSq() > maxD * maxD) continue;

            ContactManifold mChild;
            if (detectBodyBodyConvex(a, &pb, mChild)) {
                mChild.a = a;
                mChild.b = b;
                orientNormalForPair(mChild, a, b);
                out.push_back(mChild);
                if (++emitted >= maxManifoldsPerPair) return;
            }
        }
    }

    bool detectBodyBodyConvex(RigidBody* a, RigidBody* b, ContactManifold& m) {
        if (!a || !b) return false;
        if (a->isStatic && b->isStatic) return false;
        if (a->sleeping && b->sleeping) return false;

        if (!a->collider.isConvex() || !b->collider.isConvex()) return false;

        if (a->collider.type == ColliderType::Sphere && b->collider.type == ColliderType::Sphere) {
            return collideSphereSphere(a, b, m);
        }
        if (a->collider.type == ColliderType::Sphere && b->collider.type == ColliderType::Box) {
            return collideSphereBox(a, b, m);
        }
        if (a->collider.type == ColliderType::Box && b->collider.type == ColliderType::Sphere) {
            bool hit = collideSphereBox(b, a, m);
            if (!hit) return false;
            std::swap(m.a, m.b);
            m.normal = -m.normal;
            return true;
        }
        if (a->collider.type == ColliderType::Box && b->collider.type == ColliderType::Box) {
            if (collideBoxBox(a, b, m)) return true;
        }

        if (a->collider.type == ColliderType::Capsule && b->collider.type == ColliderType::Sphere) {
            return collideCapsuleSphere(a, b, m);
        }
        if (a->collider.type == ColliderType::Sphere && b->collider.type == ColliderType::Capsule) {
            bool hit = collideCapsuleSphere(b, a, m);
            if (!hit) return false;
            std::swap(m.a, m.b);
            m.normal = -m.normal;
            return true;
        }
        if (a->collider.type == ColliderType::Capsule && b->collider.type == ColliderType::Capsule) {
            return collideCapsuleCapsule(a, b, m);
        }

        GJKResult gjkRes = gjk(*a, *b);
        if (!gjkRes.hit) return false;

        EPAResult epaRes = epaPenetration(*a, *b, gjkRes.simplex);
        Vec3 normal = epaRes.normal.normalized();
        if (normal.lengthSq() < 1e-8f) {
            Vec3 delta = b->position - a->position;
            float dist = delta.length();
            normal = (dist > 1e-6f) ? (delta / dist) : Vec3{0.0f, 1.0f, 0.0f};
        }

        Vec3 ab = b->position - a->position;
        if (Vec3::dot(normal, ab) < 0.0f) {
            normal = -normal;
        }

        float penetration = epaRes.penetration;
        if (penetration < 0.0f) penetration = -penetration;
        if (penetration < 1e-5f) penetration = 1e-5f;

        Vec3 contactPoint;
        if (epaRes.hasWitness && isFiniteVec3(epaRes.pointAWorld) && isFiniteVec3(epaRes.pointBWorld)) {
            contactPoint = (epaRes.pointAWorld + epaRes.pointBWorld) * 0.5f;
        } else {
            Vec3 pA_local = a->collider.support(a->orientation.rotateInv(normal));
            Vec3 pB_local = b->collider.support(b->orientation.rotateInv(-normal));
            Vec3 pA_world = a->orientation.rotate(pA_local) + a->position;
            Vec3 pB_world = b->orientation.rotate(pB_local) + b->position;
            contactPoint = (pA_world + pB_world) * 0.5f;
        }

        if (a->sleeping && !b->sleeping) {
            std::swap(a, b);
            normal = -normal;
        }

        m.a = a;
        m.b = b;
        m.normal = normal;
        m.restitution = (a->restitution + b->restitution) * 0.5f;
        m.friction = (a->friction + b->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));

        m.count = 1;
        m.points[0].pointWorld = contactPoint;
        m.points[0].penetration = penetration;

        return true;
    }

    Vec3 invInertiaWorldMul(const RigidBody* body, const Vec3& v) const {
        Vec3 vLocal = body->orientation.rotateInv(v);
        Vec3 invI = body->getInvInertiaBody();
        Vec3 wLocal = {vLocal.x * invI.x, vLocal.y * invI.y, vLocal.z * invI.z};
        return body->orientation.rotate(wLocal);
    }

    void applyImpulseAtPoint(RigidBody* body, const Vec3& impulse, const Vec3& pointWorld, bool wake = true) {
        if (body->isStatic) return;
        if (body->mass <= 0.0001f) return;

        if (body->sleeping) {
            if (!wake) return;
            constexpr float wakeImpulse = 0.35f;
            float j2 = impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z;
            if (j2 < wakeImpulse * wakeImpulse) return;
        }

        if (wake) {
            body->sleeping = false;
            body->sleepTimer = 0.0f;
        }

        float invMass = 1.0f / body->mass;
        body->velocity = body->velocity + impulse * invMass;
        Vec3 r = pointWorld - body->position;
        Vec3 dW = invInertiaWorldMul(body, Vec3::cross(r, impulse));
        body->angularVelocity = body->angularVelocity + dW;
    }

    void applyAngularImpulse(RigidBody* body, const Vec3& angularImpulseWorld, bool wake = true) {
        if (body->isStatic) return;
        if (body->sleeping) {
            if (!wake) return;
            constexpr float wakeAngImpulse = 0.20f;
            float j2 = angularImpulseWorld.x * angularImpulseWorld.x + angularImpulseWorld.y * angularImpulseWorld.y + angularImpulseWorld.z * angularImpulseWorld.z;
            if (j2 < wakeAngImpulse * wakeAngImpulse) return;
        }
        if (wake) {
            body->sleeping = false;
            body->sleepTimer = 0.0f;
        }
        body->angularVelocity = body->angularVelocity + invInertiaWorldMul(body, angularImpulseWorld);
    }

    float characteristicContactRadius(const RigidBody* body) const {
        if (body->collider.type == ColliderType::Sphere) return body->collider.sphere.radius;
        if (body->collider.type == ColliderType::Capsule) return body->collider.capsule.radius;
        if (body->collider.type == ColliderType::Box) {
            Vec3 h = body->collider.box.halfExtents;
            float m = h.x;
            if (h.y < m) m = h.y;
            if (h.z < m) m = h.z;
            return m;
        }
        return 0.25f;
    }
};