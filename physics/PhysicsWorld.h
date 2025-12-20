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

    int solverIterations = 64;
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
        uint32_t lastSeenFrame = 0;
    };

    std::unordered_map<ContactKey, CachedImpulse, ContactKeyHash> contactCache;

    static ContactKey makeContactKey(const RigidBody* a, const RigidBody* b, const Vec3& pWorld) {
        constexpr float q = 50.0f;
        auto qi = [&](float v) -> int32_t { return (int32_t)lrintf(v * q); };
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

        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                ContactManifold m;
                if (detectBodyBody(bodies[i], bodies[j], m)) {
                    out.push_back(m);
                }
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
                for (const Vec3& v : body->collider.convexVerts) {
                    verts.push_back(body->orientation.rotate(v) + body->position);
                }
                break;
            }
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

        for (ContactManifold& m : ms) {
            for (int i = 0; i < m.count; ++i) {
                ContactKey key = makeContactKey(m.a, m.b, m.points[i].pointWorld);
                auto it = contactCache.find(key);
                if (it == contactCache.end()) continue;

                if ((m.a && m.a->sleeping) || (m.b && m.b->sleeping)) continue;

                const CachedImpulse& c = it->second;
                float n0 = c.normalImpulse * warmN;

                Vec3 t0 = {0.0f, 0.0f, 0.0f};
                float w0 = 0.0f;

                if (m.points[i].penetration > 0.0f) {
                    t0 = c.tangentImpulse * warmT;
                    w0 = c.twistImpulse * warmW;

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

    void computeRestitutionTargets(std::vector<ContactManifold>& ms) {
        constexpr float restitutionVelThreshold = 0.05f;
        for (ContactManifold& m : ms) {
            if (!m.a) continue;
            for (int i = 0; i < m.count; ++i) {
                ContactPointState& cp = m.points[i];
                Vec3 p = cp.pointWorld;

                Vec3 rA = p - m.a->position;
                Vec3 vA = m.a->velocity + cross(m.a->angularVelocity, rA);

                Vec3 vB = {0.0f, 0.0f, 0.0f};
                if (m.b) {
                    Vec3 rB = p - m.b->position;
                    vB = m.b->velocity + cross(m.b->angularVelocity, rB);
                }

                Vec3 relV = vB - vA;
                float vn = dot(relV, m.normal);
                cp.targetNormalVelocity = (vn < -restitutionVelThreshold) ? (-m.restitution * vn) : 0.0f;
            }
        }
    }

    void solveContacts(std::vector<ContactManifold>& ms, bool applyPositionCorrection) {
        constexpr float slop = 0.005f;
        constexpr float maxPositionCorrection = 0.05f;

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
                Vec3 p = cp.pointWorld;

                Vec3 rA = p - m.a->position;
                Vec3 rB = (m.b) ? (p - m.b->position) : Vec3{0.0f, 0.0f, 0.0f};

                Vec3 vA = m.a->velocity + cross(m.a->angularVelocity, rA);
                Vec3 vB = (m.b) ? (m.b->velocity + cross(m.b->angularVelocity, rB)) : Vec3{0.0f, 0.0f, 0.0f};
                Vec3 relV = vB - vA;

                float vn = dot(relV, m.normal);

                Vec3 rAxN = cross(rA, m.normal);
                float denomN = invMassA + dot(m.normal, cross(invInertiaWorldMulScaled(m.a, rAxN, invScaleA), rA));
                if (m.b) {
                    Vec3 rBxN = cross(rB, m.normal);
                    denomN += invMassB + dot(m.normal, cross(invInertiaWorldMulScaled(m.b, rBxN, invScaleB), rB));
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

                vA = m.a->velocity + cross(m.a->angularVelocity, rA);
                vB = (m.b) ? (m.b->velocity + cross(m.b->angularVelocity, rB)) : Vec3{0.0f, 0.0f, 0.0f};
                relV = vB - vA;

                Vec3 tangent = relV - m.normal * dot(relV, m.normal);
                float tLenSq = tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z;
                if (tLenSq > 1e-8f) {
                    float tLen = sqrtf(tLenSq);
                    tangent = tangent * (1.0f / tLen);

                    Vec3 rAxT = cross(rA, tangent);
                    float denomT = invMassA + dot(tangent, cross(invInertiaWorldMulScaled(m.a, rAxT, invScaleA), rA));
                    if (m.b) {
                        Vec3 rBxT = cross(rB, tangent);
                        denomT += invMassB + dot(tangent, cross(invInertiaWorldMulScaled(m.b, rBxT, invScaleB), rB));
                    }
                    if (denomT > 1e-6f) {
                        float vt = dot(relV, tangent);
                        float dT = (-vt / denomT);
                        float oldT = dot(cp.tangentImpulse, tangent);
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
                if (m.b) wRelN = dot((m.b->angularVelocity - m.a->angularVelocity), m.normal);
                else wRelN = -dot(m.a->angularVelocity, m.normal);

                if (fabsf(wRelN) > 1e-6f) {
                    float denomTw = dot(m.normal, invInertiaWorldMulScaled(m.a, m.normal, invScaleA));
                    if (m.b) denomTw += dot(m.normal, invInertiaWorldMulScaled(m.b, m.normal, invScaleB));
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
                ContactKey key = makeContactKey(m.a, m.b, m.points[i].pointWorld);
                CachedImpulse& c = contactCache[key];
                c.normalImpulse = m.points[i].normalImpulse;
                c.tangentImpulse = m.points[i].tangentImpulse;
                c.twistImpulse = m.points[i].twistImpulse;
                c.lastSeenFrame = frameId;
            }
        }
    }

    bool detectBodyBody(RigidBody* a, RigidBody* b, ContactManifold& m) {
        if (!a || !b) return false;
        if (a->isStatic && b->isStatic) return false;
        if (a->sleeping && b->sleeping) return false;

        // Get effective masses (static = infinite mass)
        float invMassA = a->isStatic ? 0.0f : 1.0f / a->mass;
        float invMassB = b->isStatic ? 0.0f : 1.0f / b->mass;
        float totalInvMass = invMassA + invMassB;
        if (totalInvMass < 0.0001f) return false;

        auto lenSq = [](const Vec3& v) -> float {
            return v.x * v.x + v.y * v.y + v.z * v.z;
        };
        auto clampf = [](float v, float lo, float hi) -> float {
            return (v < lo) ? lo : (v > hi) ? hi : v;
        };

        auto clampVec3 = [&](const Vec3& p, const Vec3& lo, const Vec3& hi) -> Vec3 {
            return {
                clampf(p.x, lo.x, hi.x),
                clampf(p.y, lo.y, hi.y),
                clampf(p.z, lo.z, hi.z),
            };
        };

        auto closestPointOnSegment = [&](const Vec3& a0, const Vec3& a1, const Vec3& p, float* outT) -> Vec3 {
            Vec3 d = a1 - a0;
            float denom = lenSq(d);
            float t = 0.0f;
            if (denom > 1e-10f) {
                t = clampf(dot(p - a0, d) / denom, 0.0f, 1.0f);
            }
            if (outT) *outT = t;
            return a0 + d * t;
        };

        auto closestPtSegmentSegment = [&](
            const Vec3& p1, const Vec3& q1,
            const Vec3& p2, const Vec3& q2,
            float* outS, float* outT,
            Vec3* outC1, Vec3* outC2
        ) {
            Vec3 d1 = q1 - p1;
            Vec3 d2 = q2 - p2;
            Vec3 r = p1 - p2;
            float a_ = dot(d1, d1);
            float e_ = dot(d2, d2);
            float f_ = dot(d2, r);

            float s = 0.0f;
            float t = 0.0f;

            if (a_ <= 1e-10f && e_ <= 1e-10f) {
                s = 0.0f;
                t = 0.0f;
            } else if (a_ <= 1e-10f) {
                s = 0.0f;
                t = clampf(f_ / e_, 0.0f, 1.0f);
            } else {
                float c_ = dot(d1, r);
                if (e_ <= 1e-10f) {
                    t = 0.0f;
                    s = clampf(-c_ / a_, 0.0f, 1.0f);
                } else {
                    float b_ = dot(d1, d2);
                    float denom = a_ * e_ - b_ * b_;
                    if (denom > 1e-10f) {
                        s = clampf((b_ * f_ - c_ * e_) / denom, 0.0f, 1.0f);
                    } else {
                        s = 0.0f;
                    }

                    t = (b_ * s + f_) / e_;

                    if (t < 0.0f) {
                        t = 0.0f;
                        s = clampf(-c_ / a_, 0.0f, 1.0f);
                    } else if (t > 1.0f) {
                        t = 1.0f;
                        s = clampf((b_ - c_) / a_, 0.0f, 1.0f);
                    }
                }
            }

            if (outS) *outS = s;
            if (outT) *outT = t;
            if (outC1) *outC1 = p1 + d1 * s;
            if (outC2) *outC2 = p2 + d2 * t;
        };

        auto capsuleSegment = [&](RigidBody* body, Vec3& outA, Vec3& outB) {
            Vec3 axis = body->orientation.rotate({0.0f, 1.0f, 0.0f});
            float hh = body->collider.capsule.halfHeight;
            outA = body->position + axis * hh;
            outB = body->position - axis * hh;
        };

        Vec3 normal = {0.0f, 1.0f, 0.0f};
        float penetration = 0.0f;
        Vec3 contactPoints[8];
        float contactPenetrations[8] = {0};
        int contactCount = 0;

        // All convex-convex handled by GJK/EPA
        if (!gjkIntersect(*a, *b)) return false;
        Vec3 delta = b->position - a->position;
        float dist = delta.length();
        Vec3 n = (dist > 1e-6f) ? (delta / dist) : Vec3{0, 1, 0};
        normal = n;
        penetration = 0.01f;
        contactPoints[0] = a->position + n * 0.1f;
        contactPenetrations[0] = penetration;
        contactCount = 1;

        if (contactCount <= 0) return false;

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

        m.count = contactCount;
        for (int i = 0; i < contactCount; ++i) {
            m.points[i].pointWorld = contactPoints[i];
            m.points[i].penetration = (contactPenetrations[i] > 0.0f) ? contactPenetrations[i] : penetration;
        }

        return true;
    }

    static float dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        };
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
        Vec3 dW = invInertiaWorldMul(body, cross(r, impulse));
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