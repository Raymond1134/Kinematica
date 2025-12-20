#pragma once

#include "../math/Vec3.h"
#include "RigidBody.h"
#include "Spring.h"

#include <vector>
#include <cmath>
#include <unordered_map>
#include <cstdint>
#include <omp.h>

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

        // Sphere
        if (body->collider.type == ColliderType::Sphere) {
            float r = body->collider.sphere.radius;
            float bottom = body->position.y - r;
            float pen = floorY - bottom;
            if (pen > 0.0f) {
                addPoint({body->position.x, floorY, body->position.z}, pen);
            }
        }

        // Capsule (pill)
        if (body->collider.type == ColliderType::Capsule) {
            float r = body->collider.capsule.radius;
            float hh = body->collider.capsule.halfHeight;

            Vec3 axis = body->orientation.rotate({0.0f, 1.0f, 0.0f});
            Vec3 a = body->position + axis * hh;
            Vec3 b = body->position - axis * hh;

            constexpr float contactDist = 0.008f;
            constexpr float speculativeSlop = 0.001f;

            auto addCandidate = [&](const Vec3& pSeg) {
                float bottom = pSeg.y - r;
                if ((bottom - floorY) <= contactDist) {
                    float pen = floorY - bottom;
                    if (pen >= -speculativeSlop) {
                        addPoint({pSeg.x, floorY, pSeg.z}, (pen > 0.0f) ? pen : 0.0f);
                    }
                }
            };

            float ay = fabsf(axis.y);
            if (ay < 0.5f && hh > 0.001f) {
                Vec3 d = a - b;
                addCandidate(b + d * 0.25f);
                addCandidate(b + d * 0.75f);
            } else {
                addCandidate((a.y < b.y) ? a : b);
            }
        }

        // Box
        if (body->collider.type == ColliderType::Box) {
            Vec3 h = body->collider.box.halfExtents;
            Vec3 X = body->orientation.rotate({1.0f, 0.0f, 0.0f});
            Vec3 Y = body->orientation.rotate({0.0f, 1.0f, 0.0f});
            Vec3 Z = body->orientation.rotate({0.0f, 0.0f, 1.0f});

            float dx = fabsf(dot(X, m.normal));
            float dy = fabsf(dot(Y, m.normal));
            float dz = fabsf(dot(Z, m.normal));
            int axis = 0;
            if (dy > dx && dy >= dz) axis = 1;
            else if (dz > dx && dz > dy) axis = 2;

            Vec3 A[3] = {X, Y, Z};
            float E[3] = {h.x, h.y, h.z};
            Vec3 axisW = A[axis];
            float extN = E[axis];
            float signToNormal = (dot(axisW, m.normal) >= 0.0f) ? 1.0f : -1.0f;
            Vec3 faceCenter = body->position + axisW * (signToNormal * extN);

            int uAxis = (axis + 1) % 3;
            int vAxis = (axis + 2) % 3;
            Vec3 U = A[uAxis];
            Vec3 V = A[vAxis];
            float extU = E[uAxis];
            float extV = E[vAxis];

            Vec3 faceVerts[4];
            faceVerts[0] = faceCenter + U * extU + V * extV;
            faceVerts[1] = faceCenter + U * extU - V * extV;
            faceVerts[2] = faceCenter - U * extU - V * extV;
            faceVerts[3] = faceCenter - U * extU + V * extV;

            // Keep this small. Large speculative contacts make boxes feel like they have suspension.
            constexpr float contactDist = 0.008f;
            constexpr float speculativeSlop = 0.001f;
            float minY = faceVerts[0].y;
            for (int i = 1; i < 4; ++i) if (faceVerts[i].y < minY) minY = faceVerts[i].y;
            if ((minY - floorY) <= contactDist) {
                for (int i = 0; i < 4; ++i) {
                    float pen = floorY - faceVerts[i].y;
                    if (pen >= -speculativeSlop) {
                        addPoint({faceVerts[i].x, floorY, faceVerts[i].z}, (pen > 0.0f) ? pen : 0.0f);
                    }
                }
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

        // Sphere-Sphere
        if (a->collider.type == ColliderType::Sphere && b->collider.type == ColliderType::Sphere) {
            float ra = a->collider.sphere.radius;
            float rb = b->collider.sphere.radius;
            Vec3 delta = b->position - a->position;
            float distSq = lenSq(delta);
            float rSum = ra + rb;
            if (distSq < rSum * rSum && distSq > 1e-8f) {
                float dist = sqrtf(distSq);
                normal = delta * (1.0f / dist);
                penetration = rSum - dist;
                contactPoints[0] = a->position + normal * ra;
                contactPenetrations[0] = penetration;
                contactCount = 1;
            } else {
                return false;
            }
        }

        // Sphere-Box
        else if ((a->collider.type == ColliderType::Sphere && b->collider.type == ColliderType::Box) || (a->collider.type == ColliderType::Box && b->collider.type == ColliderType::Sphere)) {
            bool aIsSphere = (a->collider.type == ColliderType::Sphere);
            RigidBody* sphere = aIsSphere ? a : b;
            RigidBody* box = aIsSphere ? b : a;

            float sphereRadius = sphere->collider.sphere.radius;
            Vec3 halfExt = box->collider.box.halfExtents;

            Vec3 pLocal = box->orientation.rotateInv(sphere->position - box->position);
            Vec3 closestLocal = {
                clampf(pLocal.x, -halfExt.x, halfExt.x),
                clampf(pLocal.y, -halfExt.y, halfExt.y),
                clampf(pLocal.z, -halfExt.z, halfExt.z),
            };
            Vec3 deltaLocal = pLocal - closestLocal;
            float distSqLocal = lenSq(deltaLocal);
            if (distSqLocal < sphereRadius * sphereRadius) {
                Vec3 nWorld = {0.0f, 1.0f, 0.0f};
                Vec3 closestWorld = box->position + box->orientation.rotate(closestLocal);

                if (distSqLocal < 1e-8f) {
                    float dx = halfExt.x - fabsf(pLocal.x);
                    float dy = halfExt.y - fabsf(pLocal.y);
                    float dz = halfExt.z - fabsf(pLocal.z);
                    Vec3 nLocal = {0.0f, 0.0f, 0.0f};
                    if (dx <= dy && dx <= dz) {
                        nLocal = {pLocal.x > 0 ? 1.0f : -1.0f, 0.0f, 0.0f};
                        closestLocal.x = nLocal.x * halfExt.x;
                        penetration = sphereRadius + dx;
                    } else if (dy <= dz) {
                        nLocal = {0.0f, pLocal.y > 0 ? 1.0f : -1.0f, 0.0f};
                        closestLocal.y = nLocal.y * halfExt.y;
                        penetration = sphereRadius + dy;
                    } else {
                        nLocal = {0.0f, 0.0f, pLocal.z > 0 ? 1.0f : -1.0f};
                        closestLocal.z = nLocal.z * halfExt.z;
                        penetration = sphereRadius + dz;
                    }
                    nWorld = box->orientation.rotate(nLocal);
                    closestWorld = box->position + box->orientation.rotate(closestLocal);
                } else {
                    float dist = sqrtf(distSqLocal);
                    Vec3 nLocal = deltaLocal * (1.0f / dist);
                    nWorld = box->orientation.rotate(nLocal);
                    penetration = sphereRadius - dist;
                }

                normal = nWorld;
                if (aIsSphere) normal = -normal;

                contactPoints[0] = closestWorld;
                contactPenetrations[0] = penetration;
                contactCount = 1;
            } else {
                return false;
            }
        }

        // Sphere-Capsule
        else if ((a->collider.type == ColliderType::Sphere && b->collider.type == ColliderType::Capsule) || (a->collider.type == ColliderType::Capsule && b->collider.type == ColliderType::Sphere)) {
            bool aIsSphere = (a->collider.type == ColliderType::Sphere);
            RigidBody* sphere = aIsSphere ? a : b;
            RigidBody* capsule = aIsSphere ? b : a;

            float rs = sphere->collider.sphere.radius;
            float rc = capsule->collider.capsule.radius;
            Vec3 c0, c1;
            capsuleSegment(capsule, c0, c1);

            Vec3 pCap = closestPointOnSegment(c0, c1, sphere->position, nullptr);
            Vec3 delta = sphere->position - pCap;
            float distSq = lenSq(delta);
            float rSum = rs + rc;
            if (distSq < rSum * rSum) {
                if (distSq > 1e-8f) {
                    float dist = sqrtf(distSq);
                    Vec3 nCapToSphere = delta * (1.0f / dist);
                    penetration = rSum - dist;
                    normal = aIsSphere ? (-nCapToSphere) : nCapToSphere;
                    contactPoints[0] = pCap + nCapToSphere * rc;
                    contactPenetrations[0] = penetration;
                    contactCount = 1;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }

        // Capsule-Capsule
        else if (a->collider.type == ColliderType::Capsule && b->collider.type == ColliderType::Capsule) {
            float ra = a->collider.capsule.radius;
            float rb = b->collider.capsule.radius;

            Vec3 a0, a1, b0, b1;
            capsuleSegment(a, a0, a1);
            capsuleSegment(b, b0, b1);

            Vec3 ca, cb;
            closestPtSegmentSegment(a0, a1, b0, b1, nullptr, nullptr, &ca, &cb);
            Vec3 delta = cb - ca;
            float distSq = lenSq(delta);
            float rSum = ra + rb;
            if (distSq < rSum * rSum) {
                if (distSq > 1e-8f) {
                    float dist = sqrtf(distSq);
                    normal = delta * (1.0f / dist);
                    penetration = rSum - dist;
                    contactPoints[0] = ca + normal * ra;
                    contactPenetrations[0] = penetration;
                    contactCount = 1;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }

        // Capsule-Box
        else if ((a->collider.type == ColliderType::Capsule && b->collider.type == ColliderType::Box) || (a->collider.type == ColliderType::Box && b->collider.type == ColliderType::Capsule)) {
            bool aIsCapsule = (a->collider.type == ColliderType::Capsule);
            RigidBody* capsule = aIsCapsule ? a : b;
            RigidBody* box = aIsCapsule ? b : a;

            float r = capsule->collider.capsule.radius;
            Vec3 halfExt = box->collider.box.halfExtents;

            Vec3 s0, s1;
            capsuleSegment(capsule, s0, s1);
            Vec3 s0L = box->orientation.rotateInv(s0 - box->position);
            Vec3 s1L = box->orientation.rotateInv(s1 - box->position);

            Vec3 d = s1L - s0L;
            float denom = lenSq(d);
            float t = 0.5f;
            if (denom > 1e-10f) { t = clampf(dot(-s0L, d) / denom, 0.0f, 1.0f); }

            Vec3 pSegL = s0L + d * t;
            Vec3 pBoxL = clampVec3(pSegL, {-halfExt.x, -halfExt.y, -halfExt.z}, {halfExt.x, halfExt.y, halfExt.z});
            for (int it = 0; it < 2; ++it) {
                if (denom > 1e-10f) {
                    t = clampf(dot(pBoxL - s0L, d) / denom, 0.0f, 1.0f);
                    pSegL = s0L + d * t;
                    pBoxL = clampVec3(pSegL, {-halfExt.x, -halfExt.y, -halfExt.z}, {halfExt.x, halfExt.y, halfExt.z});
                }
            }

            Vec3 deltaL = pSegL - pBoxL;
            float distSqL = lenSq(deltaL);
            if (distSqL < r * r) {
                Vec3 nWorld = {0.0f, 1.0f, 0.0f};
                Vec3 closestWorld = box->position + box->orientation.rotate(pBoxL);

                if (distSqL < 1e-8f) {
                    float dx = halfExt.x - fabsf(pSegL.x);
                    float dy = halfExt.y - fabsf(pSegL.y);
                    float dz = halfExt.z - fabsf(pSegL.z);
                    Vec3 nLocal = {0.0f, 0.0f, 0.0f};
                    if (dx <= dy && dx <= dz) {
                        nLocal = {pSegL.x > 0 ? 1.0f : -1.0f, 0.0f, 0.0f};
                        pBoxL.x = nLocal.x * halfExt.x;
                        penetration = r + dx;
                    } else if (dy <= dz) {
                        nLocal = {0.0f, pSegL.y > 0 ? 1.0f : -1.0f, 0.0f};
                        pBoxL.y = nLocal.y * halfExt.y;
                        penetration = r + dy;
                    } else {
                        nLocal = {0.0f, 0.0f, pSegL.z > 0 ? 1.0f : -1.0f};
                        pBoxL.z = nLocal.z * halfExt.z;
                        penetration = r + dz;
                    }
                    nWorld = box->orientation.rotate(nLocal);
                    closestWorld = box->position + box->orientation.rotate(pBoxL);
                } else {
                    float dist = sqrtf(distSqL);
                    Vec3 nLocal = deltaL * (1.0f / dist);
                    nWorld = box->orientation.rotate(nLocal);
                    penetration = r - dist;
                }

                normal = nWorld;
                if (aIsCapsule) normal = -normal;

                contactPoints[0] = closestWorld;
                contactPenetrations[0] = penetration;
                contactCount = 1;
            } else {
                return false;
            }
        }

        // Box-Box
        else if (a->collider.type == ColliderType::Box && b->collider.type == ColliderType::Box) {
            Vec3 aE = a->collider.box.halfExtents;
            Vec3 bE = b->collider.box.halfExtents;

            Vec3 A0 = a->orientation.rotate({1.0f, 0.0f, 0.0f});
            Vec3 A1 = a->orientation.rotate({0.0f, 1.0f, 0.0f});
            Vec3 A2 = a->orientation.rotate({0.0f, 0.0f, 1.0f});
            Vec3 B0 = b->orientation.rotate({1.0f, 0.0f, 0.0f});
            Vec3 B1 = b->orientation.rotate({0.0f, 1.0f, 0.0f});
            Vec3 B2 = b->orientation.rotate({0.0f, 0.0f, 1.0f});

            Vec3 tW = b->position - a->position;
            float tA[3] = {dot(tW, A0), dot(tW, A1), dot(tW, A2)};

            float R[3][3] = {
                {dot(A0, B0), dot(A0, B1), dot(A0, B2)},
                {dot(A1, B0), dot(A1, B1), dot(A1, B2)},
                {dot(A2, B0), dot(A2, B1), dot(A2, B2)},
            };

            float absR[3][3];
            constexpr float eps = 1e-6f;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) absR[i][j] = fabsf(R[i][j]) + eps;
            }

            float bestOverlap = 1e9f;
            Vec3 bestAxis = {1.0f, 0.0f, 0.0f};

            auto considerAxis = [&](const Vec3& axisWorld, float overlap) {
                if (overlap < bestOverlap) {
                    bestOverlap = overlap;
                    bestAxis = axisWorld;
                }
            };
            {
                float ra = aE.x;
                float rb = bE.x * absR[0][0] + bE.y * absR[0][1] + bE.z * absR[0][2];
                float dist = fabsf(tA[0]);
                if (dist > ra + rb) return false;
                considerAxis(A0, (ra + rb) - dist);
            }
            {
                float ra = aE.y;
                float rb = bE.x * absR[1][0] + bE.y * absR[1][1] + bE.z * absR[1][2];
                float dist = fabsf(tA[1]);
                if (dist > ra + rb) return false;
                considerAxis(A1, (ra + rb) - dist);
            }
            {
                float ra = aE.z;
                float rb = bE.x * absR[2][0] + bE.y * absR[2][1] + bE.z * absR[2][2];
                float dist = fabsf(tA[2]);
                if (dist > ra + rb) return false;
                considerAxis(A2, (ra + rb) - dist);
            }

            float tB[3] = {
                tA[0] * R[0][0] + tA[1] * R[1][0] + tA[2] * R[2][0],
                tA[0] * R[0][1] + tA[1] * R[1][1] + tA[2] * R[2][1],
                tA[0] * R[0][2] + tA[1] * R[1][2] + tA[2] * R[2][2],
            };
            {
                float ra = aE.x * absR[0][0] + aE.y * absR[1][0] + aE.z * absR[2][0];
                float rb = bE.x;
                float dist = fabsf(tB[0]);
                if (dist > ra + rb) return false;
                considerAxis(B0, (ra + rb) - dist);
            }
            {
                float ra = aE.x * absR[0][1] + aE.y * absR[1][1] + aE.z * absR[2][1];
                float rb = bE.y;
                float dist = fabsf(tB[1]);
                if (dist > ra + rb) return false;
                considerAxis(B1, (ra + rb) - dist);
            }
            {
                float ra = aE.x * absR[0][2] + aE.y * absR[1][2] + aE.z * absR[2][2];
                float rb = bE.z;
                float dist = fabsf(tB[2]);
                if (dist > ra + rb) return false;
                considerAxis(B2, (ra + rb) - dist);
            }

            auto testCross = [&](int i, int j, const Vec3& axisWorld) {
                float axisLenSq = lenSq(axisWorld);
                if (axisLenSq < 1e-10f) return;

                int i1 = (i + 1) % 3;
                int i2 = (i + 2) % 3;
                int j1 = (j + 1) % 3;
                int j2 = (j + 2) % 3;

                float aExt[3] = {aE.x, aE.y, aE.z};
                float bExt[3] = {bE.x, bE.y, bE.z};

                float ra = aExt[i1] * absR[i2][j] + aExt[i2] * absR[i1][j];
                float rb = bExt[j1] * absR[i][j2] + bExt[j2] * absR[i][j1];
                float dist = fabsf(tA[i2] * R[i1][j] - tA[i1] * R[i2][j]);
                if (dist > ra + rb) return;
                float overlap = (ra + rb) - dist;

                float invLen = 1.0f / sqrtf(axisLenSq);
                constexpr float crossBias = 0.001f;
                considerAxis(axisWorld * invLen, overlap + crossBias);
            };

            testCross(0, 0, cross(A0, B0));
            testCross(0, 1, cross(A0, B1));
            testCross(0, 2, cross(A0, B2));
            testCross(1, 0, cross(A1, B0));
            testCross(1, 1, cross(A1, B1));
            testCross(1, 2, cross(A1, B2));
            testCross(2, 0, cross(A2, B0));
            testCross(2, 1, cross(A2, B1));
            testCross(2, 2, cross(A2, B2));

            penetration = bestOverlap;
            normal = bestAxis;
            if (dot(tW, normal) < 0.0f) normal = -normal;

            auto supportPoint = [&](RigidBody* body, const Vec3& dirWorld) -> Vec3 {
                Vec3 e = body->collider.box.halfExtents;
                Vec3 X = body->orientation.rotate({1.0f, 0.0f, 0.0f});
                Vec3 Y = body->orientation.rotate({0.0f, 1.0f, 0.0f});
                Vec3 Z = body->orientation.rotate({0.0f, 0.0f, 1.0f});
                float sx = (dot(dirWorld, X) >= 0.0f) ? 1.0f : -1.0f;
                float sy = (dot(dirWorld, Y) >= 0.0f) ? 1.0f : -1.0f;
                float sz = (dot(dirWorld, Z) >= 0.0f) ? 1.0f : -1.0f;
                return body->position + X * (sx * e.x) + Y * (sy * e.y) + Z * (sz * e.z);
            };

            Vec3 pA = supportPoint(a, normal);
            Vec3 pB = supportPoint(b, -normal);
            Vec3 contactPoint = (pA + pB) * 0.5f;
            contactPoints[0] = contactPoint;
            contactPenetrations[0] = penetration;
            contactCount = 1;

            float a0m = fabsf(dot(normal, A0));
            float a1m = fabsf(dot(normal, A1));
            float a2m = fabsf(dot(normal, A2));
            float b0m = fabsf(dot(normal, B0));
            float b1m = fabsf(dot(normal, B1));
            float b2m = fabsf(dot(normal, B2));

            float bestFace = a0m;
            bool refIsA = true;
            int refAxis = 0;
            if (a1m > bestFace) { bestFace = a1m; refAxis = 1; }
            if (a2m > bestFace) { bestFace = a2m; refAxis = 2; }
            if (b0m > bestFace) { bestFace = b0m; refIsA = false; refAxis = 0; }
            if (b1m > bestFace) { bestFace = b1m; refIsA = false; refAxis = 1; }
            if (b2m > bestFace) { bestFace = b2m; refIsA = false; refAxis = 2; }

            constexpr float faceEps = 0.80f;
            if (bestFace >= faceEps) {
                RigidBody* ref = refIsA ? a : b;
                RigidBody* inc = refIsA ? b : a;

                Vec3 refAxes[3] = {
                    ref->orientation.rotate({1.0f, 0.0f, 0.0f}),
                    ref->orientation.rotate({0.0f, 1.0f, 0.0f}),
                    ref->orientation.rotate({0.0f, 0.0f, 1.0f}),
                };
                Vec3 incAxes[3] = {
                    inc->orientation.rotate({1.0f, 0.0f, 0.0f}),
                    inc->orientation.rotate({0.0f, 1.0f, 0.0f}),
                    inc->orientation.rotate({0.0f, 0.0f, 1.0f}),
                };

                Vec3 refE = ref->collider.box.halfExtents;
                Vec3 incE = inc->collider.box.halfExtents;

                Vec3 refToIncN = refIsA ? normal : (-normal);

                Vec3 refAxisW = refAxes[refAxis];
                float refExtN = (refAxis == 0) ? refE.x : (refAxis == 1) ? refE.y : refE.z;
                float refSign = (dot(refAxisW, refToIncN) >= 0.0f) ? 1.0f : -1.0f;
                Vec3 refPlaneN = refAxisW * refSign;
                Vec3 refFaceCenter = ref->position + refAxisW * (refSign * refExtN);

                int uAxis = (refAxis + 1) % 3;
                int vAxis = (refAxis + 2) % 3;
                Vec3 U = refAxes[uAxis];
                Vec3 V = refAxes[vAxis];
                float extU = (uAxis == 0) ? refE.x : (uAxis == 1) ? refE.y : refE.z;
                float extV = (vAxis == 0) ? refE.x : (vAxis == 1) ? refE.y : refE.z;

                float d0 = fabsf(dot(incAxes[0], refPlaneN));
                float d1 = fabsf(dot(incAxes[1], refPlaneN));
                float d2 = fabsf(dot(incAxes[2], refPlaneN));
                int incAxis = (d0 > d1) ? ((d0 > d2) ? 0 : 2) : ((d1 > d2) ? 1 : 2);

                float incExtN = (incAxis == 0) ? incE.x : (incAxis == 1) ? incE.y : incE.z;
                float incSign = (dot(incAxes[incAxis], refPlaneN) > 0.0f) ? -1.0f : 1.0f;
                Vec3 incFaceCenter = inc->position + incAxes[incAxis] * (incSign * incExtN);

                int incU = (incAxis + 1) % 3;
                int incV = (incAxis + 2) % 3;
                float incExtU = (incU == 0) ? incE.x : (incU == 1) ? incE.y : incE.z;
                float incExtV = (incV == 0) ? incE.x : (incV == 1) ? incE.y : incE.z;

                Vec3 incUAxis = incAxes[incU];
                Vec3 incVAxis = incAxes[incV];

                Vec3 poly[8];
                int polyCount = 4;
                poly[0] = incFaceCenter + incUAxis * incExtU + incVAxis * incExtV;
                poly[1] = incFaceCenter + incUAxis * incExtU - incVAxis * incExtV;
                poly[2] = incFaceCenter - incUAxis * incExtU - incVAxis * incExtV;
                poly[3] = incFaceCenter - incUAxis * incExtU + incVAxis * incExtV;

                auto clipAgainstPlane = [&](const Vec3& planeN, float planeD, Vec3* inPts, int inCount, Vec3* outPts) -> int {
                    if (inCount <= 0) return 0;
                    int outCount = 0;
                    Vec3 prev = inPts[inCount - 1];
                    float prevDist = dot(planeN, prev) - planeD;
                    for (int i = 0; i < inCount; ++i) {
                        Vec3 cur = inPts[i];
                        float curDist = dot(planeN, cur) - planeD;
                        bool curIn = (curDist <= 0.0f);
                        bool prevIn = (prevDist <= 0.0f);
                        if (curIn != prevIn) {
                            float t = prevDist / (prevDist - curDist);
                            Vec3 inter = prev + (cur - prev) * t;
                            outPts[outCount++] = inter;
                        }
                        if (curIn) outPts[outCount++] = cur;
                        prev = cur;
                        prevDist = curDist;
                    }
                    return outCount;
                };

                Vec3 tmp1[8];
                Vec3 tmp2[8];
                for (int i = 0; i < polyCount; ++i) tmp1[i] = poly[i];
                int c1 = polyCount;

                float dPosU = dot(U, refFaceCenter) + extU;
                float dNegU = dot(-U, refFaceCenter) + extU;
                float dPosV = dot(V, refFaceCenter) + extV;
                float dNegV = dot(-V, refFaceCenter) + extV;

                int c2 = clipAgainstPlane(U, dPosU, tmp1, c1, tmp2);
                c1 = clipAgainstPlane(-U, dNegU, tmp2, c2, tmp1);
                c2 = clipAgainstPlane(V, dPosV, tmp1, c1, tmp2);
                c1 = clipAgainstPlane(-V, dNegV, tmp2, c2, tmp1);

                int outCount = 0;
                for (int i = 0; i < c1 && outCount < 8; ++i) {
                    Vec3 p = tmp1[i];
                    float dist = dot(refPlaneN, p - refFaceCenter);
                    if (dist <= 0.02f) {
                        Vec3 proj = p - refPlaneN * dist;
                        contactPoints[outCount++] = proj;
                        contactPenetrations[outCount - 1] = penetration;
                    }
                }
                if (outCount > 0) {
                    contactCount = outCount;
                }
            }
        } else {
            return false;
        }

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