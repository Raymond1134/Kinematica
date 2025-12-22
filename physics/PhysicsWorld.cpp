#include "PhysicsWorld.h"
#include "collision/GJK.h"
#include "collision/EPA.h"
#include <omp.h>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdint>

PhysicsWorld::PhysicsWorld() {
    contactCache.reserve(16384);
    contactCache.max_load_factor(0.70f);
}

void PhysicsWorld::addRigidBody(RigidBody* body) { bodies.push_back(body); }

void PhysicsWorld::step(float deltaTime) {
    using clock = std::chrono::steady_clock;
    auto tStep0 = clock::now();

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

    const int nBodies = (int)bodies.size();
    const bool useOmpBodies = (nBodies >= ompMinBodiesForParallel);

    perf.bodies = nBodies;

    const float maxVelocity = 999.0f;
    const float maxAngularVelocity = 999.0f;
    const float lin = expf(-linearDampingPerSecond * deltaTime);
    const float ang = expf(-angularDampingPerSecond * deltaTime);

    auto bodyStep = [&](RigidBody* body) {
        if (!body) return;
        body->hadContactThisStep = false;
        if (body->isStatic) return;
        if (body->sleeping) return;

        body->velocity += gravity * deltaTime;
        body->position += body->velocity * deltaTime;
        body->orientation.integrateAngularVelocity(body->angularVelocity, deltaTime);

        float speedSq = body->velocity.lengthSq();
        if (speedSq > maxVelocity * maxVelocity) {
            float speed = sqrtf(speedSq);
            body->velocity = body->velocity * (maxVelocity / speed);
        }
        float angSpeedSq = body->angularVelocity.lengthSq();
        if (angSpeedSq > maxAngularVelocity * maxAngularVelocity) {
            float angSpeed = sqrtf(angSpeedSq);
            body->angularVelocity = body->angularVelocity * (maxAngularVelocity / angSpeed);
        }

        body->velocity = body->velocity * lin;
        body->angularVelocity = body->angularVelocity * ang;
    };

    if (useOmpBodies) {
        #pragma omp parallel for schedule(static)
        for (int i = 0; i < nBodies; ++i) { bodyStep(bodies[i]); }
    } else {
        for (int i = 0; i < nBodies; ++i) { bodyStep(bodies[i]); }
    }

    int awakeCount = 0;
    for (RigidBody* body : bodies) {
        if (!body) continue;
        if (body->isStatic) continue;
        if (!body->sleeping) ++awakeCount;
    }
    perf.awake = awakeCount;

    contacts.clear();

    auto tContacts0 = clock::now();
    buildContacts(contacts);
    auto tContacts1 = clock::now();
    perf.buildContactsMs = std::chrono::duration<float, std::milli>(tContacts1 - tContacts0).count();

    computeRestitutionTargets(contacts);
    warmStartContacts(contacts);

    auto tSolve0 = clock::now();
    int iterCount = solverIterations;
    if (perf.awake > 48) iterCount = std::min(iterCount, 10);
    else if (perf.awake > 24) iterCount = std::min(iterCount, 14);

    for (int pass = 0; pass < iterCount; ++pass) { solveContacts(contacts, pass == 0); }

    if (restingFrictionExtraPasses > 0) { solveRestingFriction(contacts, restingFrictionExtraPasses); }

    auto tSolve1 = clock::now();
    perf.solveMs = std::chrono::duration<float, std::milli>(tSolve1 - tSolve0).count();

    const float sleepLinear = 0.035f;
    const float sleepAngular = 0.020f;
    const float sleepTime = 0.30f;
    for (RigidBody* body: bodies) {
        if (!body) continue;
        if (body->isStatic) continue;
        if (body->sleeping) {
            body->sleepTimer = sleepTime;
            continue;
        }

        if (body->hadContactThisStep) {
            float v2 = body->velocity.lengthSq();
            float w2 = body->angularVelocity.lengthSq();
            if (v2 < contactDampingMaxSpeed * contactDampingMaxSpeed) {
                float k = expf(-contactLinearDampingPerSecond * deltaTime);
                body->velocity = body->velocity * k;
            }
            if (w2 < contactDampingMaxAngularSpeed * contactDampingMaxAngularSpeed) {
                float k = expf(-contactAngularDampingPerSecond * deltaTime);
                body->angularVelocity = body->angularVelocity * k;
            }

            if (v2 < contactRestVelKill * contactRestVelKill) {
                body->velocity = {0.0f, 0.0f, 0.0f};
                v2 = 0.0f;
            }
            if (w2 < contactRestAngVelKill * contactRestAngVelKill) {
                body->angularVelocity = {0.0f, 0.0f, 0.0f};
                w2 = 0.0f;
            }
        }

        float v2 = body->velocity.lengthSq();
        float w2 = body->angularVelocity.lengthSq();
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

    ++frameId;
    pruneContactCache();

    auto tStep1 = clock::now();
    perf.stepMs = std::chrono::duration<float, std::milli>(tStep1 - tStep0).count();
}

void PhysicsWorld::reduceManifoldTo2(ContactManifold& m) {
    if (m.count <= 2) return;

    int deepest = 0;
    float bestPen = m.points[0].penetration;
    for (int i = 1; i < m.count; ++i) {
        if (m.points[i].penetration > bestPen) {
            bestPen = m.points[i].penetration;
            deepest = i;
        }
    }

    int farthest = -1;
    float bestD2 = -1.0f;
    const Vec3 p0 = m.points[deepest].pointWorld;
    for (int i = 0; i < m.count; ++i) {
        if (i == deepest) continue;
        Vec3 d = m.points[i].pointWorld - p0;
        float d2 = d.lengthSq();
        if (d2 > bestD2) {
            bestD2 = d2;
            farthest = i;
        }
    }

    ContactPointState a = m.points[deepest];
    ContactPointState b = (farthest >= 0) ? m.points[farthest] : m.points[(deepest == 0) ? 1 : 0];
    m.points[0] = a;
    m.points[1] = b;
    m.count = 2;
}

void PhysicsWorld::reduceManifoldToMaxPen(ContactManifold& m, int maxCount) {
    if (maxCount <= 0) { m.count = 0; return; }
    if (m.count <= maxCount) return;

    int idx[8];
    for (int i = 0; i < m.count; ++i) idx[i] = i;
    const int n = m.count;

    for (int i = 0; i < maxCount; ++i) {
        int best = i;
        float bestPen = m.points[idx[i]].penetration;
        for (int j = i + 1; j < n; ++j) {
            float pen = m.points[idx[j]].penetration;
            if (pen > bestPen) {
                bestPen = pen;
                best = j;
            }
        }
        std::swap(idx[i], idx[best]);
    }

    ContactPointState kept[8];
    for (int i = 0; i < maxCount; ++i) kept[i] = m.points[idx[i]];
    for (int i = 0; i < maxCount; ++i) m.points[i] = kept[i];
    m.count = maxCount;
}

bool PhysicsWorld::isFiniteVec3(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void PhysicsWorld::buildFrictionBasis(const Vec3& n, Vec3& t1, Vec3& t2) {
    Vec3 a = (fabsf(n.y) < 0.9f) ? Vec3{0.0f, 1.0f, 0.0f} : Vec3{1.0f, 0.0f, 0.0f};
    t1 = Vec3::cross(a, n);
    float lenSq = t1.lengthSq();
    if (lenSq < 1e-12f) {
        a = Vec3{0.0f, 0.0f, 1.0f};
        t1 = Vec3::cross(a, n);
        lenSq = t1.lengthSq();
    }
    if (lenSq > 1e-12f) t1 = t1 / sqrtf(lenSq);
    else t1 = Vec3{1.0f, 0.0f, 0.0f};
    t2 = Vec3::cross(n, t1);
}

PhysicsWorld::ContactKey PhysicsWorld::makeContactKey(const RigidBody* a, const RigidBody* b, const Vec3& pWorld) {
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

const PhysicsWorld::CachedImpulse* PhysicsWorld::findCachedImpulseNear(const RigidBody* a, const RigidBody* b, const Vec3& pWorld) const {
    ContactKey key0 = makeContactKey(a, b, pWorld);
    auto it0 = contactCache.find(key0);
    if (it0 != contactCache.end()) return &it0->second;

    constexpr int r = 1;
    for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                ContactKey k = key0;
                k.qx += dx;
                k.qy += dy;
                k.qz += dz;
                auto it = contactCache.find(k);
                if (it != contactCache.end()) return &it->second;
            }
        }
    }
    return nullptr;
}

void PhysicsWorld::pruneContactCache() {
    constexpr uint32_t maxAge = 45;
    constexpr size_t maxEntries = 16384;
    for (auto it = contactCache.begin(); it != contactCache.end(); ) {
        uint32_t age = frameId - it->second.lastSeenFrame;
        if (age > maxAge) it = contactCache.erase(it);
        else ++it;
    }
    if (contactCache.size() > maxEntries) { contactCache.clear(); }
}

void PhysicsWorld::buildContacts(std::vector<ContactManifold>& out) {
    out.reserve(bodies.size() * 2);

    const int nBodies = (int)bodies.size();
    const int maxThreads = omp_get_max_threads();
    ensureContactLocals(maxThreads);
    auto& locals = contactLocals;
    const bool useOmp = (nBodies >= ompMinBodiesForParallel);

    if (enableFloor) {
        if (useOmp) {
            #pragma omp parallel
            {
                const int tid = omp_get_thread_num();
                auto& local = locals[tid];
                local.clear();

                #pragma omp for schedule(static)
                for (int i = 0; i < nBodies; ++i) {
                    RigidBody* body = bodies[i];
                    ContactManifold m;
                    if (detectFloor(body, m)) { local.push_back(m); }
                }
            }

            for (auto& local : locals) {
                out.insert(out.end(), local.begin(), local.end());
                local.clear();
            }
        } else {
            for (int i = 0; i < nBodies; ++i) {
                RigidBody* body = bodies[i];
                ContactManifold m;
                if (detectFloor(body, m)) { out.push_back(m); }
            }
        }
    }

    buildSapCandidates();

    perf.broadphasePairs = (int)sapPairs.size();

    const int nPairs = (int)sapPairs.size();
    const bool useOmpPairs = useOmp && (nPairs >= ompMinPairsForParallel);
    if (useOmpPairs) {
        int perThreadReserve = std::max(8, nPairs / std::max(1, maxThreads));
        for (auto& local : locals) {
            local.clear();
            if ((int)local.capacity() < perThreadReserve) local.reserve(perThreadReserve);
        }

        #pragma omp parallel for schedule(static)
        for (int p = 0; p < nPairs; ++p) {
            const int tid = omp_get_thread_num();
            appendBodyBodyContacts(sapPairs[p].a, sapPairs[p].b, locals[tid], epaLocals[tid]);
        }

        for (auto& local : locals) { out.insert(out.end(), local.begin(), local.end()); }
    } else {
        for (int p = 0; p < nPairs; ++p) {
            appendBodyBodyContacts(sapPairs[p].a, sapPairs[p].b, out, epaLocals[0]);
        }
    }

    perf.manifolds = (int)out.size();
}

void PhysicsWorld::ensureContactLocals(int maxThreads) {
    if ((int)contactLocals.size() != maxThreads) {
        contactLocals.clear();
        contactLocals.resize(maxThreads);
    }

    if ((int)epaLocals.size() != maxThreads) {
        epaLocals.clear();
        epaLocals.resize(maxThreads);
    }
}

void PhysicsWorld::buildSapCandidates() {
    constexpr float broadphaseMargin = 0.01f;

    auto absf3 = [](const Vec3& v) {
        return Vec3{fabsf(v.x), fabsf(v.y), fabsf(v.z)};
    };

    auto computeAabb = [&](const RigidBody* b, Vec3& outMin, Vec3& outMax) {
        switch (b->collider.type) {
            case ColliderType::Sphere: {
                float r = b->collider.sphere.radius;
                Vec3 e{r, r, r};
                outMin = b->position - e;
                outMax = b->position + e;
                return;
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
                return;
            }
            case ColliderType::Capsule: {
                float r = b->collider.capsule.radius;
                float hh = b->collider.capsule.halfHeight;
                Vec3 axis = absf3(b->orientation.rotate({0.0f, 1.0f, 0.0f}));
                Vec3 e{r + axis.x * hh, r + axis.y * hh, r + axis.z * hh};
                outMin = b->position - e;
                outMax = b->position + e;
                return;
            }
            case ColliderType::Convex:
            case ColliderType::Compound:
            case ColliderType::Mesh: {
                float r = b->collider.boundingRadius();
                Vec3 e{r, r, r};
                outMin = b->position - e;
                outMax = b->position + e;
                return;
            }
        }
    };

    sapEntries.clear();
    sapEntries.reserve(bodies.size());
    for (RigidBody* body : bodies) {
        if (!body) continue;
        Vec3 mn, mx;
        computeAabb(body, mn, mx);
        SapEntry e;
        e.minX = mn.x - broadphaseMargin;
        e.maxX = mx.x + broadphaseMargin;
        e.minY = mn.y - broadphaseMargin;
        e.maxY = mx.y + broadphaseMargin;
        e.minZ = mn.z - broadphaseMargin;
        e.maxZ = mx.z + broadphaseMargin;
        e.body = body;
        sapEntries.push_back(e);
    }

    std::sort(sapEntries.begin(), sapEntries.end(), [](const SapEntry& a, const SapEntry& b) { return a.minX < b.minX; });

    sapPairs.clear();
    sapPairs.reserve(sapEntries.size() * 2);

    sapActive.clear();
    if (sapActive.capacity() < 128) sapActive.reserve(128);

    for (int i = 0; i < (int)sapEntries.size(); ++i) {
        const SapEntry& cur = sapEntries[i];
        RigidBody* bi = cur.body;
        if (!bi) continue;
        int write = 0;
        for (int k = 0; k < (int)sapActive.size(); ++k) {
            if (sapEntries[sapActive[k]].maxX >= cur.minX) { sapActive[write++] = sapActive[k]; }
        }
        sapActive.resize(write);

        for (int idx : sapActive) {
            const SapEntry& other = sapEntries[idx];
            RigidBody* bj = other.body;
            if (!bj) continue;
            if (bi == bj) continue;
            if (bi->isStatic && bj->isStatic) continue;
            if (bi->sleeping && bj->sleeping) continue;

            if (cur.maxY < other.minY || cur.minY > other.maxY) continue;
            if (cur.maxZ < other.minZ || cur.minZ > other.maxZ) continue;

            sapPairs.push_back({bi, bj});
        }

        sapActive.push_back(i);
    }
}

bool PhysicsWorld::detectFloor(RigidBody* body, ContactManifold& m) {
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

    auto testVertex = [&](const Vec3& vWorld) {
        float pen = floorY - vWorld.y;
        if (pen > 0.0f) { addPoint({vWorld.x, floorY, vWorld.z}, pen); }
    };

    switch (body->collider.type) {
        case ColliderType::Sphere: {
            float r = body->collider.sphere.radius;
            testVertex(body->position + Vec3{0, -r, 0});
            break;
        }
        case ColliderType::Capsule: {
            float r = body->collider.capsule.radius;
            float hh = body->collider.capsule.halfHeight;
            Vec3 axis = body->orientation.rotate({0.0f, 1.0f, 0.0f});
            testVertex(body->position + axis * hh + Vec3{0, -r, 0});
            testVertex(body->position - axis * hh + Vec3{0, -r, 0});
            break;
        }
        case ColliderType::Box: {
            Vec3 he = body->collider.box.halfExtents;
            for (int dx = -1; dx <= 1; dx += 2) {
                for (int dy = -1; dy <= 1; dy += 2) {
                    for (int dz = -1; dz <= 1; dz += 2) {
                        Vec3 local = {he.x * dx, he.y * dy, he.z * dz};
                        testVertex(body->orientation.rotate(local) + body->position);
                    }
                }
            }
            break;
        }
        case ColliderType::Convex: {
            if (body->collider.polyhedron) {
                for (const Vec3& v : body->collider.polyhedron->verts) {
                    testVertex(body->orientation.rotate(v) + body->position);
                }
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
                        testVertex(pChildWorld + Vec3{0, -r, 0});
                        break;
                    }
                    case ColliderType::Capsule: {
                        float r = child.collider.capsule.radius;
                        float hh = child.collider.capsule.halfHeight;
                        Vec3 axis = qChildWorld.rotate({0.0f, 1.0f, 0.0f});
                        testVertex(pChildWorld + axis * hh + Vec3{0, -r, 0});
                        testVertex(pChildWorld - axis * hh + Vec3{0, -r, 0});
                        break;
                    }
                    case ColliderType::Box: {
                        Vec3 he = child.collider.box.halfExtents;
                        for (int dx = -1; dx <= 1; dx += 2) {
                            for (int dy = -1; dy <= 1; dy += 2) {
                                for (int dz = -1; dz <= 1; dz += 2) {
                                    Vec3 local = {he.x * dx, he.y * dy, he.z * dz};
                                    testVertex(qChildWorld.rotate(local) + pChildWorld);
                                }
                            }
                        }
                        break;
                    }
                    case ColliderType::Convex: {
                        if (child.collider.polyhedron) {
                            for (const Vec3& v : child.collider.polyhedron->verts) {
                                testVertex(qChildWorld.rotate(v) + pChildWorld);
                            }
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

    if (body->collider.type == ColliderType::Box) {
        if (m.count > 4) reduceManifoldToMaxPen(m, 4);
    } else if (body->collider.type == ColliderType::Convex || body->collider.type == ColliderType::Compound) {
        if (m.count > 4) reduceManifoldToMaxPen(m, 4);
    } else {
        if (m.count > 2) reduceManifoldTo2(m);
    }
    return m.count > 0;
}

void PhysicsWorld::warmStartContacts(std::vector<ContactManifold>& ms) {
    constexpr float warmN = 0.90f;
    constexpr float warmT = 0.55f;
    constexpr float warmW = 0.55f;

    constexpr float minNormalAlign = 0.80f;
    constexpr float minTwistAlign = 0.95f;

    for (ContactManifold& m : ms) {
        for (int i = 0; i < m.count; ++i) {
            if (!isFiniteVec3(m.points[i].pointWorld)) continue;
            const CachedImpulse* cPtr = findCachedImpulseNear(m.a, m.b, m.points[i].pointWorld);
            bool swapped = false;

            if (!cPtr && m.b) {
                cPtr = findCachedImpulseNear(m.b, m.a, m.points[i].pointWorld);
                swapped = (cPtr != nullptr);
            }

            if (!cPtr) continue;

            if ((m.a && m.a->sleeping) || (m.b && m.b->sleeping)) continue;

            const CachedImpulse& c = *cPtr;

            Vec3 cachedNormal = c.normalWorld;
            if (swapped) cachedNormal = -cachedNormal;

            float nAlign = Vec3::dot(cachedNormal, m.normal);
            if (nAlign < minNormalAlign) continue;

            float n0 = c.normalImpulse * warmN;

            Vec3 t0 = {0.0f, 0.0f, 0.0f};
            float w0 = 0.0f;

            if (m.points[i].penetration > 0.0f) {
                t0 = c.tangentImpulse * warmT;
                t0 = t0 - m.normal * Vec3::dot(t0, m.normal);

                if (nAlign >= minTwistAlign) { w0 = c.twistImpulse * warmW; }

                float tLenSq = t0.x * t0.x + t0.y * t0.y + t0.z * t0.z;
                if (tLenSq > 1e-12f) {
                    float tLen = sqrtf(tLenSq);
                    float maxT = m.friction * n0;
                    if (tLen > maxT && maxT > 0.0f) { t0 = t0 * (maxT / tLen); }
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

float PhysicsWorld::clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void PhysicsWorld::getBoxAxes(const RigidBody* b, Vec3 outAxes[3]) {
    outAxes[0] = b->orientation.rotate({1.0f, 0.0f, 0.0f});
    outAxes[1] = b->orientation.rotate({0.0f, 1.0f, 0.0f});
    outAxes[2] = b->orientation.rotate({0.0f, 0.0f, 1.0f});
}

float PhysicsWorld::projectBoxRadiusOnAxis(const RigidBody* b, const Vec3 axes[3], const Vec3& axisUnit) {
    const Vec3 he = b->collider.box.halfExtents;
    float r = 0.0f;
    r += he.x * fabsf(Vec3::dot(axisUnit, axes[0]));
    r += he.y * fabsf(Vec3::dot(axisUnit, axes[1]));
    r += he.z * fabsf(Vec3::dot(axisUnit, axes[2]));
    return r;
}

int PhysicsWorld::clipPolygonToPlane(const Vec3* inVerts, int inCount, Vec3* outVerts, const Vec3& n, float d) {
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
        if (curIn) { outVerts[outCount++] = cur; }

        prev = cur;
        prevDist = curDist;
    }
    return outCount;
}

float PhysicsWorld::closestPtSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, float& s, float& t, Vec3& c1, Vec3& c2) {
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

bool PhysicsWorld::collideSphereSphere(RigidBody* a, RigidBody* b, ContactManifold& m) {
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

bool PhysicsWorld::collideSphereBox(RigidBody* sphereBody, RigidBody* boxBody, ContactManifold& m) {
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

bool PhysicsWorld::collideCapsuleSphere(RigidBody* capsuleBody, RigidBody* sphereBody, ContactManifold& m) {
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

bool PhysicsWorld::collideCapsuleCapsule(RigidBody* a, RigidBody* b, ContactManifold& m) {
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

bool PhysicsWorld::collideBoxBox(RigidBody* a, RigidBody* b, ContactManifold& m) {
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
        float ra = heA.x * fabsf(Vec3::dot(n, Ax[0])) + heA.y * fabsf(Vec3::dot(n, Ax[1])) + heA.z * fabsf(Vec3::dot(n, Ax[2]));
        float rb = heB.x * fabsf(Vec3::dot(n, Bx[0])) + heB.y * fabsf(Vec3::dot(n, Bx[1])) + heB.z * fabsf(Vec3::dot(n, Bx[2]));
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

    for (int i = 0; i < 3; ++i) { if (!testAxis(Ax[i], 0, i)) return false; }
    for (int i = 0; i < 3; ++i) { if (!testAxis(Bx[i], 1, i)) return false; }

    int crossIndex = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 c = Vec3::cross(Ax[i], Bx[j]);
            if (!testAxis(c, 2, crossIndex)) return false;
            ++crossIndex;
        }
    }

    if (Vec3::dot(bestAxis, d) < 0.0f) { bestAxis = -bestAxis; }
    if (bestOverlap < 1e-5f) bestOverlap = 1e-5f;

    if (bestAxisType == 2) {
        const int ai = bestAxisIndex / 3;
        const int bi = bestAxisIndex % 3;
        const Vec3 u = Ax[ai];
        const Vec3 v = Bx[bi];
        const int a1 = (ai + 1) % 3;
        const int a2 = (ai + 2) % 3;
        const int b1 = (bi + 1) % 3;
        const int b2 = (bi + 2) % 3;

        auto heComp = [](const Vec3& he, int axis) -> float {
            return (axis == 0) ? he.x : (axis == 1 ? he.y : he.z);
        };

        const float sA1 = (Vec3::dot(bestAxis, Ax[a1]) >= 0.0f) ? 1.0f : -1.0f;
        const float sA2 = (Vec3::dot(bestAxis, Ax[a2]) >= 0.0f) ? 1.0f : -1.0f;
        const float sB1 = (Vec3::dot(-bestAxis, Bx[b1]) >= 0.0f) ? 1.0f : -1.0f;
        const float sB2 = (Vec3::dot(-bestAxis, Bx[b2]) >= 0.0f) ? 1.0f : -1.0f;

        Vec3 aEdgeCenter = a->position + Ax[a1] * (sA1 * heComp(heA, a1)) + Ax[a2] * (sA2 * heComp(heA, a2));
        Vec3 bEdgeCenter = b->position + Bx[b1] * (sB1 * heComp(heB, b1)) + Bx[b2] * (sB2 * heComp(heB, b2));
        Vec3 a0 = aEdgeCenter - u * heComp(heA, ai);
        Vec3 a1p = aEdgeCenter + u * heComp(heA, ai);
        Vec3 b0 = bEdgeCenter - v * heComp(heB, bi);
        Vec3 b1p = bEdgeCenter + v * heComp(heB, bi);

        float s = 0.0f, t = 0.0f;
        Vec3 c1, c2;
        (void)closestPtSegmentSegment(a0, a1p, b0, b1p, s, t, c1, c2);

        Vec3 p = (c1 + c2) * 0.5f;

        m.a = a;
        m.b = b;
        m.normal = bestAxis;
        m.restitution = (a->restitution + b->restitution) * 0.5f;
        m.friction = (a->friction + b->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));

        m.count = 1;
        m.points[0].pointWorld = p;
        m.points[0].penetration = bestOverlap;
        m.points[0].targetNormalVelocity = 0.0f;
        return true;
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

    m.a = a;
    m.b = b;
    m.normal = bestAxis;
    m.restitution = (a->restitution + b->restitution) * 0.5f;
    m.friction = (a->friction + b->friction) * 0.5f;
    m.frictionTwist = m.friction * 0.35f;
    m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));
    return true;
}

void PhysicsWorld::computeRestitutionTargets(std::vector<ContactManifold>& ms) {
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

void PhysicsWorld::solveContacts(std::vector<ContactManifold>& ms, bool applyPositionCorrection) {
    constexpr float slop = 0.005f;
    constexpr float maxPositionCorrection = 0.08f;
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
            if (!m.b->sleeping) { m.b->sleepTimer = 0.0f; }
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
            if (!enableSplitImpulse && cp.penetration > slop && currentDt > 1e-6f) {
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

            Vec3 t1, t2;
            PhysicsWorld::buildFrictionBasis(m.normal, t1, t2);

            float oldT1 = Vec3::dot(cp.tangentImpulse, t1);
            float oldT2 = Vec3::dot(cp.tangentImpulse, t2);
            float newT1 = oldT1;
            float newT2 = oldT2;

            {
                Vec3 rAxT = Vec3::cross(rA, t1);
                float denomT = invMassA + Vec3::dot(t1, Vec3::cross(invInertiaWorldMulScaled(m.a, rAxT, invScaleA), rA));
                if (m.b) {
                    Vec3 rBxT = Vec3::cross(rB, t1);
                    denomT += invMassB + Vec3::dot(t1, Vec3::cross(invInertiaWorldMulScaled(m.b, rBxT, invScaleB), rB));
                }
                if (denomT > 1e-6f) {
                    float vt = Vec3::dot(relV, t1);
                    newT1 = oldT1 + (-vt / denomT);
                }
            }
            {
                Vec3 rAxT = Vec3::cross(rA, t2);
                float denomT = invMassA + Vec3::dot(t2, Vec3::cross(invInertiaWorldMulScaled(m.a, rAxT, invScaleA), rA));
                if (m.b) {
                    Vec3 rBxT = Vec3::cross(rB, t2);
                    denomT += invMassB + Vec3::dot(t2, Vec3::cross(invInertiaWorldMulScaled(m.b, rBxT, invScaleB), rB));
                }
                if (denomT > 1e-6f) {
                    float vt = Vec3::dot(relV, t2);
                    newT2 = oldT2 + (-vt / denomT);
                }
            }
            float nRef = (cp.normalImpulse > supportImpulse) ? cp.normalImpulse : supportImpulse;
            float maxF = m.friction * nRef;
            float magSq = newT1 * newT1 + newT2 * newT2;
            if (magSq > maxF * maxF && magSq > 1e-12f) {
                float invMag = 1.0f / sqrtf(magSq);
                float s = maxF * invMag;
                newT1 *= s;
                newT2 *= s;
            }
            float dT1 = newT1 - oldT1;
            float dT2 = newT2 - oldT2;
            if (fabsf(dT1) > 0.0f || fabsf(dT2) > 0.0f) {
                Vec3 Jt = t1 * dT1 + t2 * dT2;
                cp.tangentImpulse = t1 * newT1 + t2 * newT2;
                if (m.b) {
                    applyImpulseAtPoint(m.a, -Jt, p);
                    applyImpulseAtPoint(m.b, Jt, p);
                } else {
                    applyImpulseAtPoint(m.a, -Jt, p);
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

void PhysicsWorld::solveRestingFriction(std::vector<ContactManifold>& ms, int passes) {
    if (passes <= 0) return;

    constexpr float slop = 0.005f;
    constexpr float shockLowerInvScale = 0.25f;
    constexpr float shockNormalY = 0.55f;

    for (int pass = 0; pass < passes; ++pass) {
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
            float vASq = m.a->velocity.lengthSq();
            float wASq = m.a->angularVelocity.lengthSq();
            if (vASq > restingMaxBodySpeed * restingMaxBodySpeed) continue;
            if (wASq > restingMaxBodyAngularSpeed * restingMaxBodyAngularSpeed) continue;
            
            if (m.b) {
                float vBSq = m.b->velocity.lengthSq();
                float wBSq = m.b->angularVelocity.lengthSq();
                if (vBSq > restingMaxBodySpeed * restingMaxBodySpeed) continue;
                if (wBSq > restingMaxBodyAngularSpeed * restingMaxBodyAngularSpeed) continue;
            }

            float maxPen = 0.0f;
            for (int i = 0; i < m.count; ++i) if (m.points[i].penetration > maxPen) maxPen = m.points[i].penetration;
            if (maxPen <= slop) continue;

            auto invInertiaWorldMulScaled = [&](const RigidBody* body, const Vec3& v, float invScale) -> Vec3 {
                if (!body) return {0.0f, 0.0f, 0.0f};
                if (body->isStatic) return {0.0f, 0.0f, 0.0f};
                if (body->sleeping) return {0.0f, 0.0f, 0.0f};
                return invInertiaWorldMul(body, v) * invScale;
            };

            float invCount = 1.0f / (float)m.count;
            float supportImpulse = 0.0f;
            if (totalInvMass > 1e-6f) { supportImpulse = (fabsf(gravity.y) * currentDt / totalInvMass) * invCount;}

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
                if (fabsf(vn) > restingMaxNormalSpeed) continue;

                Vec3 t1, t2;
                PhysicsWorld::buildFrictionBasis(m.normal, t1, t2);

                float oldT1 = Vec3::dot(cp.tangentImpulse, t1);
                float oldT2 = Vec3::dot(cp.tangentImpulse, t2);
                float newT1 = oldT1;
                float newT2 = oldT2;

                {
                    Vec3 rAxT = Vec3::cross(rA, t1);
                    float denomT = invMassA + Vec3::dot(t1, Vec3::cross(invInertiaWorldMulScaled(m.a, rAxT, invScaleA), rA));
                    if (m.b) {
                        Vec3 rBxT = Vec3::cross(rB, t1);
                        denomT += invMassB + Vec3::dot(t1, Vec3::cross(invInertiaWorldMulScaled(m.b, rBxT, invScaleB), rB));
                    }
                    if (denomT > 1e-6f) {
                        float vt = Vec3::dot(relV, t1);
                        newT1 = oldT1 + (-vt / denomT);
                    }
                }

                {
                    Vec3 rAxT = Vec3::cross(rA, t2);
                    float denomT = invMassA + Vec3::dot(t2, Vec3::cross(invInertiaWorldMulScaled(m.a, rAxT, invScaleA), rA));
                    if (m.b) {
                        Vec3 rBxT = Vec3::cross(rB, t2);
                        denomT += invMassB + Vec3::dot(t2, Vec3::cross(invInertiaWorldMulScaled(m.b, rBxT, invScaleB), rB));
                    }
                    if (denomT > 1e-6f) {
                        float vt = Vec3::dot(relV, t2);
                        newT2 = oldT2 + (-vt / denomT);
                    }
                }

                float nRef = (cp.normalImpulse > supportImpulse) ? cp.normalImpulse : supportImpulse;
                float maxF = m.friction * nRef;
                float magSq = newT1 * newT1 + newT2 * newT2;
                if (magSq > maxF * maxF && magSq > 1e-12f) {
                    float invMag = 1.0f / sqrtf(magSq);
                    float s = maxF * invMag;
                    newT1 *= s;
                    newT2 *= s;
                }

                float dT1 = newT1 - oldT1;
                float dT2 = newT2 - oldT2;
                if (dT1 != 0.0f || dT2 != 0.0f) {
                    Vec3 Jt = t1 * dT1 + t2 * dT2;
                    cp.tangentImpulse = t1 * newT1 + t2 * newT2;
                    if (m.b) {
                        applyImpulseAtPoint(m.a, -Jt, p);
                        applyImpulseAtPoint(m.b, Jt, p);
                    } else {
                        applyImpulseAtPoint(m.a, -Jt, p);
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
                        float maxTw = m.frictionTwist * nRef * m.patchR;
                        if (newW > maxTw) newW = maxTw;
                        if (newW < -maxTw) newW = -maxTw;
                        float dTw = newW - oldW;
                        cp.twistImpulse = newW;

                        if (dTw != 0.0f) {
                            if (m.b) {
                                applyAngularImpulse(m.a, m.normal * (-dTw));
                                applyAngularImpulse(m.b, m.normal * (dTw));
                            } else {
                                applyAngularImpulse(m.a, m.normal * (-dTw));
                            }
                        }
                    }
                }
            }
        }
    }
}

RigidBody PhysicsWorld::makeChildProxyBody(const RigidBody* parent, const CompoundChild& child) {
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

void PhysicsWorld::orientNormalForPair(ContactManifold& m, const RigidBody* a, const RigidBody* b) {
    Vec3 ab = b->position - a->position;
    if (Vec3::dot(m.normal, ab) < 0.0f) m.normal = -m.normal;
}

void PhysicsWorld::appendBodyBodyContacts(RigidBody* a, RigidBody* b, std::vector<ContactManifold>& out, EPAScratch& epaScratch) {
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
        if (detectBodyBodyConvex(a, b, m, epaScratch)) out.push_back(m);
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
                if (detectBodyBodyConvex(&pa, &pb, mChild, epaScratch)) {
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
            if (detectBodyBodyConvex(&pa, b, mChild, epaScratch)) {
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
        if (detectBodyBodyConvex(a, &pb, mChild, epaScratch)) {
            mChild.a = a;
            mChild.b = b;
            orientNormalForPair(mChild, a, b);
            out.push_back(mChild);
            if (++emitted >= maxManifoldsPerPair) return;
        }
    }
}

bool PhysicsWorld::detectBodyBodyConvex(RigidBody* a, RigidBody* b, ContactManifold& m, EPAScratch& epaScratch) {
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

    EPAResult epaRes = epaPenetration(*a, *b, gjkRes.simplex, epaScratch);
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

Vec3 PhysicsWorld::invInertiaWorldMul(const RigidBody* body, const Vec3& v) const {
    Vec3 vLocal = body->orientation.rotateInv(v);
    Vec3 invI = body->getInvInertiaBody();
    Vec3 wLocal = {vLocal.x * invI.x, vLocal.y * invI.y, vLocal.z * invI.z};
    return body->orientation.rotate(wLocal);
}

void PhysicsWorld::applyImpulseAtPoint(RigidBody* body, const Vec3& impulse, const Vec3& pointWorld, bool wake) {
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

void PhysicsWorld::applyAngularImpulse(RigidBody* body, const Vec3& angularImpulseWorld, bool wake) {
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

float PhysicsWorld::characteristicContactRadius(const RigidBody* body) const {
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