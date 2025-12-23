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

    const float dt = std::max(0.0f, deltaTime);
    const int substeps = enableCCD ? computeCcdSubsteps(dt) : 1;
    const float subDt = (substeps > 0) ? (dt / (float)substeps) : dt;

    perf.stepMs = 0.0f;
    perf.buildContactsMs = 0.0f;
    perf.solveMs = 0.0f;

    for (int i = 0; i < substeps; ++i) {
        auto t0 = clock::now();
        stepSubstep(subDt, i == (substeps - 1));
        auto t1 = clock::now();
        perf.stepMs += std::chrono::duration<float, std::milli>(t1 - t0).count();
    }

    pruneContactCache();

    auto tStep1 = clock::now();
    (void)tStep1;
}

int PhysicsWorld::computeCcdSubsteps(float deltaTime) const {
    if (deltaTime <= 0.0f) return 1;
    if (ccdMaxSubsteps <= 1) return 1;
    if (ccdMaxTranslationFraction <= 0.0f) return 1;

    int required = 1;
    for (RigidBody* body : bodies) {
        if (!body) continue;
        if (body->isStatic) continue;
        if (body->sleeping) continue;

        float r = characteristicContactRadius(body);
        r = std::max(r, ccdMinCharacteristicRadius);

        float maxMove = r * ccdMaxTranslationFraction;
        float speed = std::sqrt(body->velocity.lengthSq());
        float angSpeed = std::sqrt(body->angularVelocity.lengthSq());

        // This is still not TOI CCD.
        float travel = (speed + angSpeed * r) * deltaTime;
        if (maxMove > 1e-6f && travel > maxMove) {
            int s = (int)std::ceil(travel / maxMove);
            if (s > required) required = s;
            if (required >= ccdMaxSubsteps) return ccdMaxSubsteps;
        }
    }

    required = std::clamp(required, 1, ccdMaxSubsteps);
    return required;
}

void PhysicsWorld::stepSubstep(float deltaTime, bool isFinalSubstep) {
    using clock = std::chrono::steady_clock;

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
    perf.buildContactsMs += std::chrono::duration<float, std::milli>(tContacts1 - tContacts0).count();

    computeRestitutionTargets(contacts);
    warmStartContacts(contacts);

    auto tSolve0 = clock::now();
    int iterCount = solverIterations;

    for (int pass = 0; pass < iterCount; ++pass) { solveContacts(contacts, pass == 0); }

    if (restingFrictionExtraPasses > 0) { solveRestingFriction(contacts, restingFrictionExtraPasses); }

    auto tSolve1 = clock::now();
    perf.solveMs += std::chrono::duration<float, std::milli>(tSolve1 - tSolve0).count();

    const float sleepLinear = 0.025f;
    const float sleepAngular = 0.015f;
    const float sleepTime = 0.35f;
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
    (void)isFinalSubstep;
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
    float bestD2 = 0.0f;
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

    if (farthest < 0 || bestD2 < 1e-6f) {
        farthest = (deepest == 0) ? 1 : 0;
    }

    ContactPointState a = m.points[deepest];
    ContactPointState b = m.points[farthest];
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
    constexpr float q = 80.0f;
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
                    return;
                }
                float r = b->collider.boundingRadius();
                Vec3 e{r, r, r};
                outMin = b->position - e;
                outMax = b->position + e;
                return;
            }
            case ColliderType::Mesh: {
                if (b->collider.mesh) {
                    const TriangleMesh& mesh = *static_cast<const TriangleMesh*>(b->collider.mesh.get());
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
                    return;
                }
                float r = b->collider.boundingRadius();
                Vec3 e{r, r, r};
                outMin = b->position - e;
                outMax = b->position + e;
                return;
            }
            case ColliderType::Compound: {
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
                    case ColliderType::Compound:
                        assert(false && "Nested compound child encountered; createCompound() should flatten compounds");
                        break;
                    case ColliderType::Mesh:
                        assert(false && "Mesh colliders are not supported for floor contacts");
                        break;
                }
            }
            break;
        }
        case ColliderType::Mesh: {
            if (!body->collider.mesh) break;
            for (const Vec3& vLocal : body->collider.mesh->vertices) {
                testVertex(body->orientation.rotate(vLocal) + body->position);
            }
            break;
        }
    }

    if (body->collider.type == ColliderType::Box) {
        if (m.count > 4) reduceManifoldToMaxPen(m, 4);
    } else if (body->collider.type == ColliderType::Convex || body->collider.type == ColliderType::Compound || body->collider.type == ColliderType::Mesh) {
        if (m.count > 4) reduceManifoldToMaxPen(m, 4);
    } else {
        if (m.count > 2) reduceManifoldTo2(m);
    }
    return m.count > 0;
}

void PhysicsWorld::warmStartContacts(std::vector<ContactManifold>& ms) {
    constexpr float warmN = 1.0f;
    constexpr float warmT = 1.0f;
    constexpr float warmW = 1.0f;

    constexpr float minNormalAlign = 0.90f;
    constexpr float minTwistAlign = 0.98f;

    for (ContactManifold& m : ms) {
        const bool hasMesh =
            (m.a && m.a->collider.type == ColliderType::Mesh) ||
            (m.b && m.b->collider.type == ColliderType::Mesh);
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

            if (m.points[i].penetration > -0.01f) {
                if (!hasMesh) {
                    t0 = c.tangentImpulse * warmT;
                    t0 = t0 - m.normal * Vec3::dot(t0, m.normal);

                    if (nAlign >= minTwistAlign) { w0 = c.twistImpulse * warmW; }
                }

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

void PhysicsWorld::appendMeshBoxManifolds(const RigidBody* meshBody, const RigidBody* boxBody, std::vector<ContactManifold>& out, int maxManifolds) const {
    if (!meshBody || !boxBody) return;
    if (maxManifolds <= 0) return;
    if (!meshBody->collider.mesh) return;

    const TriangleMesh& mesh = *meshBody->collider.mesh;
    const bool upwardOnly = (mesh.flags & TriangleMesh::CollideUpwardOnly) != 0;
    const float minUp = 0.10f;
    const Vec3 he = boxBody->collider.box.halfExtents;

    Vec3 boxCenterM = meshBody->orientation.rotateInv(boxBody->position - meshBody->position);

    Vec3 boxAxesW[3] = {
        boxBody->orientation.rotate({1.0f, 0.0f, 0.0f}),
        boxBody->orientation.rotate({0.0f, 1.0f, 0.0f}),
        boxBody->orientation.rotate({0.0f, 0.0f, 1.0f}),
    };
    Vec3 boxAxesM[3] = {
        meshBody->orientation.rotateInv(boxAxesW[0]),
        meshBody->orientation.rotateInv(boxAxesW[1]),
        meshBody->orientation.rotateInv(boxAxesW[2]),
    };

    float r = he.length();
    Vec3 qMin = boxCenterM - Vec3{r, r, r};
    Vec3 qMax = boxCenterM + Vec3{r, r, r};

    std::vector<uint32_t> triIds;
    triIds.reserve(512);
    mesh.queryAabb(qMin, qMax, triIds, 4096);
    if (triIds.empty()) return;

    Vec3 vertsW[8];
    Vec3 vertsM[8];
    int vIdx = 0;
    for (int dx = -1; dx <= 1; dx += 2) {
        for (int dy = -1; dy <= 1; dy += 2) {
            for (int dz = -1; dz <= 1; dz += 2) {
                Vec3 local = {he.x * dx, he.y * dy, he.z * dz};
                Vec3 wv = boxBody->orientation.rotate(local) + boxBody->position;
                vertsW[vIdx] = wv;
                vertsM[vIdx] = meshBody->orientation.rotateInv(wv - meshBody->position);
                ++vIdx;
            }
        }
    }

    struct Cluster {
        Vec3 nLocal;
        Vec3 a0, b0, c0;
        Vec3 a1, b1, c1;
        bool hasSecond = false;
        float pen = 0.0f;
        float support = 0.0f;
    };
    std::vector<Cluster> clusters;
    clusters.reserve(8);

    auto tryAddCluster = [&](const Vec3& nLocalIn, const Vec3& a, const Vec3& b, const Vec3& c, float pen, float support) {
        constexpr float sameNormalCos = 0.98f;
        for (Cluster& cl : clusters) {
            if (Vec3::dot(cl.nLocal, nLocalIn) >= sameNormalCos) {
                if (!cl.hasSecond) {
                    cl.a1 = a; cl.b1 = b; cl.c1 = c;
                    cl.hasSecond = true;
                }
                if (pen > cl.pen) {
                    cl.pen = pen;
                    cl.a0 = a; cl.b0 = b; cl.c0 = c;
                }
                if (support > cl.support) cl.support = support;
                return;
            }
        }
        if ((int)clusters.size() < 8) {
            Cluster cl;
            cl.nLocal = nLocalIn;
            cl.a0 = a; cl.b0 = b; cl.c0 = c;
            cl.pen = pen;
            cl.support = support;
            clusters.push_back(cl);
        } else {
            int worst = 0;
            for (int i = 1; i < (int)clusters.size(); ++i) {
                if (clusters[i].support < clusters[worst].support) worst = i;
                else if (clusters[i].support == clusters[worst].support && clusters[i].pen < clusters[worst].pen) worst = i;
            }
            if (support > clusters[worst].support || (support == clusters[worst].support && pen > clusters[worst].pen)) {
                clusters[worst].nLocal = nLocalIn;
                clusters[worst].a0 = a; clusters[worst].b0 = b; clusters[worst].c0 = c;
                clusters[worst].hasSecond = false;
                clusters[worst].pen = pen;
                clusters[worst].support = support;
            }
        }
    };

    for (uint32_t tid : triIds) {
        if (tid >= mesh.tris.size()) continue;
        const TriangleMeshTri& t = mesh.tris[tid];
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];

        Vec3 n = Vec3::cross(b - a, c - a);
        float nLenSq = n.lengthSq();
        if (nLenSq < 1e-12f) continue;

        if (upwardOnly) {
            Vec3 nTriLocal = n / sqrtf(nLenSq);
            if (nTriLocal.y <= minUp) continue;
        }

        Vec3 nLocal = n / sqrtf(nLenSq);

        float rOnN =
            he.x * fabsf(Vec3::dot(nLocal, boxAxesM[0])) +
            he.y * fabsf(Vec3::dot(nLocal, boxAxesM[1])) +
            he.z * fabsf(Vec3::dot(nLocal, boxAxesM[2]));

        Vec3 triCent = (a + b + c) * (1.0f / 3.0f);
        if (Vec3::dot(nLocal, boxCenterM - triCent) < 0.0f) nLocal = -nLocal;

        float distCenter = Vec3::dot(boxCenterM - a, nLocal);
        if (distCenter >= rOnN) continue;
        float pen = (rOnN - distCenter);
        if (pen < 1e-5f) pen = 1e-5f;

        Vec3 nWorld = meshBody->orientation.rotate(nLocal);
        float support = std::max(0.0f, nWorld.y);
        tryAddCluster(nLocal, a, b, c, pen, support);
    }

    if (clusters.empty()) return;

    bool hasSupport = false;
    for (const Cluster& cl : clusters) {
        if (cl.support >= 0.05f) { hasSupport = true; break; }
    }
    if (hasSupport) {
        int w = 0;
        for (int i = 0; i < (int)clusters.size(); ++i) {
            if (clusters[i].support >= 0.05f) clusters[w++] = clusters[i];
        }
        clusters.resize(w);
        if (clusters.empty()) return;
    }

    std::sort(clusters.begin(), clusters.end(), [](const Cluster& x, const Cluster& y) {
        if (x.support != y.support) return x.support > y.support;
        return x.pen > y.pen;
    });

    const int emitCount = std::min(maxManifolds, (int)clusters.size());
    for (int ci = 0; ci < emitCount; ++ci) {
        const Cluster& cl = clusters[ci];
        const Vec3 nLocal = cl.nLocal;
        Vec3 nWorld = meshBody->orientation.rotate(nLocal);

        ContactManifold m;
        m.a = const_cast<RigidBody*>(meshBody);
        m.b = const_cast<RigidBody*>(boxBody);
        m.normal = nWorld;
        m.restitution = (meshBody->restitution + boxBody->restitution) * 0.5f;
        m.friction = (meshBody->friction + boxBody->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(meshBody) + characteristicContactRadius(boxBody));
        m.count = 0;

        struct VHit { float dist = 0.0f; int i = -1; };
        VHit bestV[4];
        for (int i = 0; i < 4; ++i) { bestV[i].dist = 0.0f; bestV[i].i = -1; }

        auto insertIfBetter = [&](float dist, int vi) {
            if (dist >= 0.0f) return;
            for (int k = 0; k < 4; ++k) if (bestV[k].i == vi) return;
            int slot = -1;
            for (int k = 0; k < 4; ++k) if (bestV[k].i < 0) { slot = k; break; }
            if (slot < 0) {
                float worstDist = bestV[0].dist;
                int worstIdx = 0;
                for (int k = 1; k < 4; ++k) {
                    if (bestV[k].dist > worstDist) { worstDist = bestV[k].dist; worstIdx = k; }
                }
                if (dist >= worstDist) return;
                slot = worstIdx;
            }
            bestV[slot].dist = dist;
            bestV[slot].i = vi;
        };

        for (int vi = 0; vi < 8; ++vi) {
            float dist = Vec3::dot(vertsM[vi] - cl.a0, nLocal);
            insertIfBetter(dist, vi);
        }

        for (int k = 0; k < 4 && m.count < 4; ++k) {
            if (bestV[k].i < 0) continue;
            int vi = bestV[k].i;
            float dist = bestV[k].dist;
            float pen = -dist;
            if (pen < 1e-5f) pen = 1e-5f;

            Vec3 vM = vertsM[vi];
            Vec3 pPlane = vM - nLocal * dist;
            Vec3 triPtM0 = pointInTri(pPlane, cl.a0, cl.b0, cl.c0) ? pPlane : closestPtPointTriangle(pPlane, cl.a0, cl.b0, cl.c0);
            Vec3 bestTriPtM = triPtM0;
            float bestD2 = (triPtM0 - pPlane).lengthSq();
            if (cl.hasSecond) {
                Vec3 triPtM1 = pointInTri(pPlane, cl.a1, cl.b1, cl.c1) ? pPlane : closestPtPointTriangle(pPlane, cl.a1, cl.b1, cl.c1);
                float d2 = (triPtM1 - pPlane).lengthSq();
                if (d2 < bestD2) { bestD2 = d2; bestTriPtM = triPtM1; }
            }

            Vec3 pTriWorld = meshBody->orientation.rotate(bestTriPtM) + meshBody->position;
            Vec3 pBoxWorld = vertsW[vi];
            Vec3 p = (pTriWorld + pBoxWorld) * 0.5f;

            m.points[m.count].pointWorld = p;
            m.points[m.count].penetration = pen;
            m.points[m.count].targetNormalVelocity = 0.0f;
            ++m.count;
        }

        if (m.count > 0) {
            out.push_back(m);
        }
    }
}

void PhysicsWorld::appendMeshConvexManifolds(const RigidBody* meshBody, const RigidBody* convexBody, std::vector<ContactManifold>& out, int maxManifolds) const {
    if (!meshBody || !convexBody) return;
    if (maxManifolds <= 0) return;
    if (!meshBody->collider.mesh) return;
    if (convexBody->collider.type != ColliderType::Convex) return;
    if (!convexBody->collider.polyhedron) return;
    if (convexBody->collider.polyhedron->verts.empty()) return;

    const TriangleMesh& mesh = *meshBody->collider.mesh;
    const bool upwardOnly = (mesh.flags & TriangleMesh::CollideUpwardOnly) != 0;
    const float minUp = 0.10f;

    Vec3 convexCenterM = meshBody->orientation.rotateInv(convexBody->position - meshBody->position);

    float r = convexBody->collider.boundingRadius();
    r = std::max(r, 0.01f);
    Vec3 qMin = convexCenterM - Vec3{r, r, r};
    Vec3 qMax = convexCenterM + Vec3{r, r, r};

    std::vector<uint32_t> triIds;
    triIds.reserve(512);
    mesh.queryAabb(qMin, qMax, triIds, 4096);
    if (triIds.empty()) return;

    const PolyhedronShape& poly = *convexBody->collider.polyhedron;
    std::vector<Vec3> vertsW;
    std::vector<Vec3> vertsM;
    vertsW.reserve(poly.verts.size());
    vertsM.reserve(poly.verts.size());
    for (const Vec3& vLocal : poly.verts) {
        Vec3 wv = convexBody->orientation.rotate(vLocal) + convexBody->position;
        vertsW.push_back(wv);
        vertsM.push_back(meshBody->orientation.rotateInv(wv - meshBody->position));
    }

    struct Cluster {
        Vec3 nLocal;
        Vec3 a, b, c;
        float pen = 0.0f;
    };
    std::vector<Cluster> clusters;
    clusters.reserve(8);

    auto tryAddCluster = [&](const Vec3& nLocalIn, const Vec3& a, const Vec3& b, const Vec3& c, float pen) {
        constexpr float sameNormalCos = 0.98f;
        for (Cluster& cl : clusters) {
            if (Vec3::dot(cl.nLocal, nLocalIn) >= sameNormalCos) {
                if (pen > cl.pen) {
                    cl.pen = pen;
                    cl.a = a; cl.b = b; cl.c = c;
                }
                return;
            }
        }
        if ((int)clusters.size() < 8) {
            Cluster cl;
            cl.nLocal = nLocalIn;
            cl.a = a; cl.b = b; cl.c = c;
            cl.pen = pen;
            clusters.push_back(cl);
        } else {
            int worst = 0;
            for (int i = 1; i < (int)clusters.size(); ++i) {
                if (clusters[i].pen < clusters[worst].pen) worst = i;
            }
            if (pen > clusters[worst].pen) {
                clusters[worst].nLocal = nLocalIn;
                clusters[worst].a = a; clusters[worst].b = b; clusters[worst].c = c;
                clusters[worst].pen = pen;
            }
        }
    };

    for (uint32_t tid : triIds) {
        if (tid >= mesh.tris.size()) continue;
        const TriangleMeshTri& t = mesh.tris[tid];
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];

        Vec3 n = Vec3::cross(b - a, c - a);
        float nLenSq = n.lengthSq();
        if (nLenSq < 1e-12f) continue;

        Vec3 nTriLocal = n / sqrtf(nLenSq);
        if (upwardOnly && nTriLocal.y <= minUp) continue;

        Vec3 nLocal = nTriLocal;
        Vec3 triCent = (a + b + c) * (1.0f / 3.0f);
        if (Vec3::dot(nLocal, convexCenterM - triCent) < 0.0f) nLocal = -nLocal;

        Vec3 nWorld = meshBody->orientation.rotate(nLocal);
        Vec3 dirConvLocal = convexBody->orientation.rotateInv(nWorld);
        float dirLenSq = dirConvLocal.lengthSq();
        if (dirLenSq < 1e-12f) continue;
        dirConvLocal = dirConvLocal / sqrtf(dirLenSq);

        Vec3 supportLocal = convexBody->collider.support(dirConvLocal);
        float rOnN = Vec3::dot(supportLocal, dirConvLocal);

        float distCenter = Vec3::dot(convexCenterM - a, nLocal);
        if (distCenter >= rOnN) continue;
        float pen = (rOnN - distCenter);
        if (pen < 1e-5f) pen = 1e-5f;

        tryAddCluster(nLocal, a, b, c, pen);
    }

    if (clusters.empty()) return;
    std::sort(clusters.begin(), clusters.end(), [](const Cluster& x, const Cluster& y) { return x.pen > y.pen; });

    const int emitCount = std::min(maxManifolds, (int)clusters.size());
    for (int ci = 0; ci < emitCount; ++ci) {
        const Cluster& cl = clusters[ci];
        const Vec3 nLocal = cl.nLocal;
        Vec3 nWorld = meshBody->orientation.rotate(nLocal);

        ContactManifold m;
        m.a = const_cast<RigidBody*>(meshBody);
        m.b = const_cast<RigidBody*>(convexBody);
        m.normal = nWorld;
        m.restitution = (meshBody->restitution + convexBody->restitution) * 0.5f;
        m.friction = (meshBody->friction + convexBody->friction) * 0.5f;
        m.frictionTwist = m.friction * 0.35f;
        m.patchR = 0.5f * (characteristicContactRadius(meshBody) + characteristicContactRadius(convexBody));
        m.count = 0;

        struct VHit { float dist = 0.0f; int i = -1; };
        VHit bestV[4];
        for (int i = 0; i < 4; ++i) { bestV[i].dist = 0.0f; bestV[i].i = -1; }

        auto insertIfBetter = [&](float dist, int vi) {
            if (dist >= 0.0f) return;
            for (int k = 0; k < 4; ++k) if (bestV[k].i == vi) return;
            int slot = -1;
            for (int k = 0; k < 4; ++k) if (bestV[k].i < 0) { slot = k; break; }
            if (slot < 0) {
                float worstDist = bestV[0].dist;
                int worstIdx = 0;
                for (int k = 1; k < 4; ++k) {
                    if (bestV[k].dist > worstDist) { worstDist = bestV[k].dist; worstIdx = k; }
                }
                if (dist >= worstDist) return;
                slot = worstIdx;
            }
            bestV[slot].dist = dist;
            bestV[slot].i = vi;
        };

        const int vCount = (int)vertsM.size();
        for (int vi = 0; vi < vCount; ++vi) {
            float dist = Vec3::dot(vertsM[vi] - cl.a, nLocal);
            insertIfBetter(dist, vi);
        }

        for (int k = 0; k < 4 && m.count < 4; ++k) {
            if (bestV[k].i < 0) continue;
            int vi = bestV[k].i;
            float dist = bestV[k].dist;
            float pen = -dist;
            if (pen < 1e-5f) pen = 1e-5f;
            if (pen > 0.5f) pen = 0.5f;

            Vec3 vM = vertsM[vi];
            Vec3 pPlane = vM - nLocal * dist;
            Vec3 triPtM = pointInTri(pPlane, cl.a, cl.b, cl.c) ? pPlane : closestPtPointTriangle(pPlane, cl.a, cl.b, cl.c);

            Vec3 pTriWorld = meshBody->orientation.rotate(triPtM) + meshBody->position;
            Vec3 pConvWorld = vertsW[vi];
            Vec3 p = (pTriWorld + pConvWorld) * 0.5f;

            m.points[m.count].pointWorld = p;
            m.points[m.count].penetration = pen;
            m.points[m.count].targetNormalVelocity = 0.0f;
            ++m.count;
        }

        if (m.count > 0) {
            out.push_back(m);
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
    Vec3 n = bestAxis;
    Vec3 refAxes[3], incAxes[3];
    getBoxAxes(ref, refAxes);
    getBoxAxes(inc, incAxes);
    Vec3 refHe = ref->collider.box.halfExtents;
    Vec3 incHe = inc->collider.box.halfExtents;
    int refAxis = bestAxisIndex;
    Vec3 refOut = (ref == a) ? n : -n;
    float refSign = (Vec3::dot(refOut, refAxes[refAxis]) >= 0.0f) ? 1.0f : -1.0f;
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

    float planeD = Vec3::dot(n, refCenter);
    m.count = 0;
    for (int i = 0; i < count && m.count < 4; ++i) {
        Vec3 p = tmp1[i];
        float distToPlane = Vec3::dot(n, p) - planeD;
        float pen = -distToPlane;
        if (pen <= 0.0f) continue;
        Vec3 projected = p - n * distToPlane;
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
    m.normal = n;
    m.restitution = (a->restitution + b->restitution) * 0.5f;
    m.friction = (a->friction + b->friction) * 0.5f;
    m.frictionTwist = m.friction * 0.35f;
    m.patchR = 0.5f * (characteristicContactRadius(a) + characteristicContactRadius(b));
    return true;
}

void PhysicsWorld::computeRestitutionTargets(std::vector<ContactManifold>& ms) {
    constexpr float restitutionVelThreshold = 0.08f;
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
            if (vn < -restitutionVelThreshold) {
                cp.targetNormalVelocity = -m.restitution * vn;
            } else if (vn < 0.0f) {
                float speedRatio = (-vn) / restitutionVelThreshold;
                float restitutionScale = speedRatio * speedRatio;
                cp.targetNormalVelocity = -m.restitution * vn * restitutionScale;
            } else {
                cp.targetNormalVelocity = 0.0f;
            }
        }
    }
}

void PhysicsWorld::solveContacts(std::vector<ContactManifold>& ms, bool applyPositionCorrection) {
    constexpr float slop = 0.005f;
    constexpr float shockLowerInvScale = 0.35f;
    constexpr float shockNormalY = 0.65f;
    const float beta = 0.2f;

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

        (void)applyPositionCorrection;

        Vec3 t1, t2;
        PhysicsWorld::buildFrictionBasis(m.normal, t1, t2);

        float baumgarteFactor = beta / currentDt;

        for (int ci = 0; ci < m.count; ++ci) {
            ContactPointState& cp = m.points[ci];
            if (!isFiniteVec3(cp.pointWorld)) continue;
            Vec3 p = cp.pointWorld;
            Vec3 rA = p - m.a->position;
            Vec3 rB = (m.b) ? (p - m.b->position) : Vec3{0.0f, 0.0f, 0.0f};
            {
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

                if (denomN > 1e-6f) {
                    float bias = 0.0f;
                    if (cp.penetration > slop) {
                        bias = baumgarteFactor * (cp.penetration - slop);
                        if (bias > 2.0f) bias = 2.0f;
                    }

                    float desired = -(vn - cp.targetNormalVelocity - bias) / denomN;
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
                }
            }
            {
                Vec3 vA = m.a->velocity + Vec3::cross(m.a->angularVelocity, rA);
                Vec3 vB = (m.b) ? (m.b->velocity + Vec3::cross(m.b->angularVelocity, rB)) : Vec3{0.0f, 0.0f, 0.0f};
                Vec3 relV = vB - vA;

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
                float nRef = cp.normalImpulse;
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
            }
            {
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
                        
                        float nRef = cp.normalImpulse;
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
    constexpr float shockLowerInvScale = 0.35f;
    constexpr float shockNormalY = 0.65f;

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

            Vec3 t1, t2;
            PhysicsWorld::buildFrictionBasis(m.normal, t1, t2);

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
    if (a->isStatic && !b->isStatic) std::swap(a, b);

    constexpr int maxManifoldsPerPair = 4;
    int emitted = 0;

    constexpr float childBroadphaseMargin = 0.02f;

    bool aCompound = (a->collider.type == ColliderType::Compound);
    bool bCompound = (b->collider.type == ColliderType::Compound);

    if (!aCompound && !bCompound) {
        if (a->collider.type == ColliderType::Mesh || b->collider.type == ColliderType::Mesh) {
            appendBodyBodyMeshContacts(a, b, out);
            return;
        }

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

bool PhysicsWorld::pointInTri(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 v0 = b - a;
    Vec3 v1 = c - a;
    Vec3 v2 = p - a;
    float d00 = Vec3::dot(v0, v0);
    float d01 = Vec3::dot(v0, v1);
    float d11 = Vec3::dot(v1, v1);
    float d20 = Vec3::dot(v2, v0);
    float d21 = Vec3::dot(v2, v1);
    float denom = d00 * d11 - d01 * d01;
    if (fabsf(denom) < 1e-12f) return false;
    float inv = 1.0f / denom;
    float v = (d11 * d20 - d01 * d21) * inv;
    float w = (d00 * d21 - d01 * d20) * inv;
    float u = 1.0f - v - w;
    const float eps = -1e-4f;
    return u >= eps && v >= eps && w >= eps;
}

Vec3 PhysicsWorld::closestPtPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = p - a;
    float d1 = Vec3::dot(ab, ap);
    float d2 = Vec3::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    Vec3 bp = p - b;
    float d3 = Vec3::dot(ab, bp);
    float d4 = Vec3::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    Vec3 cp = p - c;
    float d5 = Vec3::dot(ab, cp);
    float d6 = Vec3::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return a + ab * v + ac * w;
}

void PhysicsWorld::appendBodyBodyMeshContacts(RigidBody* a, RigidBody* b, std::vector<ContactManifold>& out) {
    if (!a || !b) return;
    if (a->isStatic && b->isStatic) return;
    if (a->sleeping && b->sleeping) return;
    if (a->collider.type == ColliderType::Compound || b->collider.type == ColliderType::Compound) return;

    constexpr int maxManifolds = 4;
    int emitted = 0;

    // Mesh vs Mesh is not implemented yet
    if (a->collider.type == ColliderType::Mesh && b->collider.type == ColliderType::Mesh) return;

    auto emitForOrder = [&](RigidBody* meshBody, RigidBody* otherBody, bool otherIsB) {
        bool hit = false;
        if (otherBody->collider.type == ColliderType::Sphere) {
            ContactManifold m;
            hit = collideMeshSphere(meshBody, otherBody, m);
            if (!hit) return;
            if (otherIsB) {
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            } else {
                std::swap(m.a, m.b);
                m.normal = -m.normal;
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            }
            ++emitted;
        } else if (otherBody->collider.type == ColliderType::Capsule) {
            ContactManifold m;
            hit = collideMeshCapsule(meshBody, otherBody, m);
            if (!hit) return;
            if (otherIsB) {
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            } else {
                std::swap(m.a, m.b);
                m.normal = -m.normal;
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            }
            ++emitted;
        } else if (otherBody->collider.type == ColliderType::Box) {
            ContactManifold m;
            hit = collideMeshBox(meshBody, otherBody, m);
            if (!hit) return;

            if (otherIsB) {
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            } else {
                std::swap(m.a, m.b);
                m.normal = -m.normal;
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            }

            ++emitted;
            return;
        } else if (otherBody->collider.type == ColliderType::Convex) {
            std::vector<ContactManifold> tmp;
            tmp.reserve(4);
            appendMeshConvexManifolds(meshBody, otherBody, tmp, maxManifolds - emitted);
            if (tmp.empty()) return;

            for (ContactManifold& m : tmp) {
                if (otherIsB) {
                    m.a = a;
                    m.b = b;
                    orientNormalForPair(m, m.a, m.b);
                    out.push_back(m);
                } else {
                    std::swap(m.a, m.b);
                    m.normal = -m.normal;
                    m.a = a;
                    m.b = b;
                    orientNormalForPair(m, m.a, m.b);
                    out.push_back(m);
                }
                if (++emitted >= maxManifolds) break;
            }
            return;
        } else {
            ContactManifold m;
            RigidBody pseudo = *otherBody;
            pseudo.collider = Collider::createSphere(otherBody->collider.boundingRadius());
            hit = collideMeshSphere(meshBody, &pseudo, m);
            if (hit) {
                m.restitution = (otherBody->restitution + meshBody->restitution) * 0.5f;
                m.friction = (otherBody->friction + meshBody->friction) * 0.5f;
                m.frictionTwist = m.friction * 0.35f;
            }
            if (!hit) return;
            if (otherIsB) {
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            } else {
                std::swap(m.a, m.b);
                m.normal = -m.normal;
                m.a = a;
                m.b = b;
                orientNormalForPair(m, m.a, m.b);
                out.push_back(m);
            }
            ++emitted;
        }
    };

    if (a->collider.type == ColliderType::Mesh) {
        emitForOrder(a, b, true);
        return;
    }
    if (b->collider.type == ColliderType::Mesh) {
        emitForOrder(b, a, false);
        return;
    }
    (void)maxManifolds;
    (void)emitted;
}

bool PhysicsWorld::collideMeshSphere(const RigidBody* meshBody, const RigidBody* sphereBody, ContactManifold& m) const {
    if (!meshBody || !sphereBody) return false;
    if (!meshBody->collider.mesh) return false;

    const TriangleMesh& mesh = *meshBody->collider.mesh;
    const bool upwardOnly = (mesh.flags & TriangleMesh::CollideUpwardOnly) != 0;
    const float minUp = 0.10f;
    const float r = sphereBody->collider.sphere.radius;

    Vec3 cLocal = meshBody->orientation.rotateInv(sphereBody->position - meshBody->position);
    Vec3 qMin = cLocal - Vec3{r, r, r};
    Vec3 qMax = cLocal + Vec3{r, r, r};

    std::vector<uint32_t> triIds;
    triIds.reserve(128);
    mesh.queryAabb(qMin, qMax, triIds, 1024);
    if (triIds.empty()) return false;

    float bestPen = 0.0f;
    Vec3 bestNLocal = {0.0f, 1.0f, 0.0f};
    Vec3 bestPtLocal = {0.0f, 0.0f, 0.0f};
    Vec3 bestSpherePtLocal = {0.0f, 0.0f, 0.0f};
    bool hit = false;

    for (uint32_t tid : triIds) {
        if (tid >= mesh.tris.size()) continue;
        const TriangleMeshTri& t = mesh.tris[tid];
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];

        if (upwardOnly) {
            Vec3 nTri = Vec3::cross(b - a, c - a);
            float nLenSq = nTri.lengthSq();
            if (nLenSq < 1e-12f) continue;
            Vec3 nTriLocal = nTri / sqrtf(nLenSq);
            if (nTriLocal.y <= minUp) continue;
        }

        Vec3 p = closestPtPointTriangle(cLocal, a, b, c);
        Vec3 d = cLocal - p;
        float distSq = d.lengthSq();
        if (distSq > r * r) continue;

        float dist = sqrtf(std::max(0.0f, distSq));
        float pen = r - dist;
        if (pen < 1e-5f) pen = 1e-5f;
        if (pen > r * 0.9f) pen = r * 0.9f;

        Vec3 nLocal;
        if (dist > 1e-5f) {
            nLocal = d / dist;
        } else {
            Vec3 n = Vec3::cross(b - a, c - a);
            if (n.lengthSq() < 1e-12f) nLocal = {0.0f, 1.0f, 0.0f};
            else {
                nLocal = n.normalized();
                if (Vec3::dot(nLocal, cLocal - (a + b + c) * (1.0f / 3.0f)) < 0.0f) nLocal = -nLocal;
            }
        }

        if (!hit || pen > bestPen) {
            hit = true;
            bestPen = pen;
            bestNLocal = nLocal;
            bestPtLocal = p;
            bestSpherePtLocal = cLocal - nLocal * r;
        }
    }

    if (!hit) return false;

    Vec3 nWorld = meshBody->orientation.rotate(bestNLocal);
    Vec3 pMeshWorld = meshBody->orientation.rotate(bestPtLocal) + meshBody->position;
    Vec3 pSphereWorld = meshBody->orientation.rotate(bestSpherePtLocal) + meshBody->position;
    Vec3 p = (pMeshWorld + pSphereWorld) * 0.5f;

    m.a = const_cast<RigidBody*>(meshBody);
    m.b = const_cast<RigidBody*>(sphereBody);
    m.normal = nWorld;
    m.restitution = (meshBody->restitution + sphereBody->restitution) * 0.5f;
    m.friction = (meshBody->friction + sphereBody->friction) * 0.5f;
    m.frictionTwist = m.friction * 0.35f;
    m.patchR = 0.5f * (characteristicContactRadius(meshBody) + characteristicContactRadius(sphereBody));
    m.count = 1;
    m.points[0].pointWorld = p;
    m.points[0].penetration = bestPen;
    m.points[0].targetNormalVelocity = 0.0f;
    return true;
}

bool PhysicsWorld::collideMeshCapsule(const RigidBody* meshBody, const RigidBody* capsuleBody, ContactManifold& m) const {
    if (!meshBody || !capsuleBody) return false;
    if (!meshBody->collider.mesh) return false;
    const TriangleMesh& mesh = *meshBody->collider.mesh;
    const bool upwardOnly = (mesh.flags & TriangleMesh::CollideUpwardOnly) != 0;
    const float minUp = 0.10f;

    const float rc = capsuleBody->collider.capsule.radius;
    const float hh = capsuleBody->collider.capsule.halfHeight;

    Vec3 axisW = capsuleBody->orientation.rotate({0.0f, 1.0f, 0.0f});
    Vec3 p0W = capsuleBody->position + axisW * hh;
    Vec3 p1W = capsuleBody->position - axisW * hh;

    Vec3 p0 = meshBody->orientation.rotateInv(p0W - meshBody->position);
    Vec3 p1 = meshBody->orientation.rotateInv(p1W - meshBody->position);

    Vec3 segMin{std::min(p0.x, p1.x), std::min(p0.y, p1.y), std::min(p0.z, p1.z)};
    Vec3 segMax{std::max(p0.x, p1.x), std::max(p0.y, p1.y), std::max(p0.z, p1.z)};
    Vec3 qMin = segMin - Vec3{rc, rc, rc};
    Vec3 qMax = segMax + Vec3{rc, rc, rc};

    std::vector<uint32_t> triIds;
    triIds.reserve(256);
    mesh.queryAabb(qMin, qMax, triIds, 2048);
    if (triIds.empty()) return false;

    float bestPen = 0.0f;
    Vec3 bestNLocal = {0.0f, 1.0f, 0.0f};
    Vec3 bestTriPt = {0.0f, 0.0f, 0.0f};
    Vec3 bestSegPt = {0.0f, 0.0f, 0.0f};
    bool hit = false;

    auto closestPtSegmentTriangle = [&](const Vec3& s0, const Vec3& s1, const Vec3& a, const Vec3& b, const Vec3& c, Vec3& outSeg, Vec3& outTri) -> float {
        Vec3 n = Vec3::cross(b - a, c - a);
        float nLenSq = n.lengthSq();
        if (nLenSq > 1e-12f) {
            n = n / sqrtf(nLenSq);
            float d0 = Vec3::dot(s0 - a, n);
            float d1 = Vec3::dot(s1 - a, n);
            if ((d0 <= 0.0f && d1 >= 0.0f) || (d0 >= 0.0f && d1 <= 0.0f)) {
                float t = (fabsf(d0 - d1) > 1e-12f) ? (d0 / (d0 - d1)) : 0.0f;
                t = clampf(t, 0.0f, 1.0f);
                Vec3 p = s0 + (s1 - s0) * t;
                if (pointInTri(p, a, b, c)) {
                    outSeg = p;
                    outTri = p;
                    return 0.0f;
                }
            }
        }

        float best = 1e30f;
        Vec3 segBest, triBest;
        Vec3 pA = closestPtPointTriangle(s0, a, b, c);
        float dA = (s0 - pA).lengthSq();
        best = dA; segBest = s0; triBest = pA;
        Vec3 pB = closestPtPointTriangle(s1, a, b, c);
        float dB = (s1 - pB).lengthSq();

        if (dB < best) { best = dB; segBest = s1; triBest = pB; }

        float ss, tt;
        Vec3 c1, c2;
        float dE;

        dE = closestPtSegmentSegment(s0, s1, a, b, ss, tt, c1, c2);
        if (dE < best) { best = dE; segBest = c1; triBest = c2; }
        dE = closestPtSegmentSegment(s0, s1, b, c, ss, tt, c1, c2);
        if (dE < best) { best = dE; segBest = c1; triBest = c2; }
        dE = closestPtSegmentSegment(s0, s1, c, a, ss, tt, c1, c2);
        if (dE < best) { best = dE; segBest = c1; triBest = c2; }

        outSeg = segBest;
        outTri = triBest;
        return best;
    };

    for (uint32_t tid : triIds) {
        if (tid >= mesh.tris.size()) continue;
        const TriangleMeshTri& t = mesh.tris[tid];
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];

        if (upwardOnly) {
            Vec3 nTri = Vec3::cross(b - a, c - a);
            float nLenSq = nTri.lengthSq();
            if (nLenSq < 1e-12f) continue;
            Vec3 nTriLocal = nTri / sqrtf(nLenSq);
            if (nTriLocal.y <= minUp) continue;
        }

        Vec3 segPt, triPt;
        float distSq = closestPtSegmentTriangle(p0, p1, a, b, c, segPt, triPt);
        if (distSq > rc * rc) continue;
        float dist = sqrtf(std::max(0.0f, distSq));
        float pen = rc - dist;
        if (pen < 1e-5f) pen = 1e-5f;
        if (pen > rc * 0.9f) pen = rc * 0.9f;

        Vec3 d = segPt - triPt;
        Vec3 nLocal;
        if (dist > 1e-5f) nLocal = d / dist;
        else {
            Vec3 n = Vec3::cross(b - a, c - a);
            if (n.lengthSq() < 1e-12f) nLocal = {0.0f, 1.0f, 0.0f};
            else {
                nLocal = n.normalized();
                if (Vec3::dot(nLocal, segPt - (a + b + c) * (1.0f / 3.0f)) < 0.0f) nLocal = -nLocal;
            }
        }

        if (!hit || pen > bestPen) {
            hit = true;
            bestPen = pen;
            bestNLocal = nLocal;
            bestTriPt = triPt;
            bestSegPt = segPt;
        }
    }

    if (!hit) return false;

    Vec3 nWorld = meshBody->orientation.rotate(bestNLocal);
    Vec3 pTriWorld = meshBody->orientation.rotate(bestTriPt) + meshBody->position;
    Vec3 pSegWorld = meshBody->orientation.rotate(bestSegPt) + meshBody->position;
    Vec3 pCapWorld = pSegWorld - nWorld * rc;
    Vec3 p = (pTriWorld + pCapWorld) * 0.5f;

    m.a = const_cast<RigidBody*>(meshBody);
    m.b = const_cast<RigidBody*>(capsuleBody);
    m.normal = nWorld;
    m.restitution = (meshBody->restitution + capsuleBody->restitution) * 0.5f;
    m.friction = (meshBody->friction + capsuleBody->friction) * 0.5f;
    m.frictionTwist = m.friction * 0.35f;
    m.patchR = 0.5f * (characteristicContactRadius(meshBody) + characteristicContactRadius(capsuleBody));
    m.count = 1;
    m.points[0].pointWorld = p;
    m.points[0].penetration = bestPen;
    m.points[0].targetNormalVelocity = 0.0f;
    return true;
}

bool PhysicsWorld::collideMeshBox(const RigidBody* meshBody, const RigidBody* boxBody, ContactManifold& m) const {
    if (!meshBody || !boxBody) return false;
    if (!meshBody->collider.mesh) return false;
    const TriangleMesh& mesh = *meshBody->collider.mesh;

    const Vec3 he = boxBody->collider.box.halfExtents;

    Vec3 boxCenterLocal = meshBody->orientation.rotateInv(boxBody->position - meshBody->position);
    Quat relRot = meshBody->orientation.conjugate() * boxBody->orientation;
    
    Vec3 axes[3] = {
        relRot.rotate({1.0f, 0.0f, 0.0f}),
        relRot.rotate({0.0f, 1.0f, 0.0f}),
        relRot.rotate({0.0f, 0.0f, 1.0f})
    };
    
    Vec3 absAxes[3] = {
        {fabsf(axes[0].x), fabsf(axes[0].y), fabsf(axes[0].z)},
        {fabsf(axes[1].x), fabsf(axes[1].y), fabsf(axes[1].z)},
        {fabsf(axes[2].x), fabsf(axes[2].y), fabsf(axes[2].z)}
    };
    
    Vec3 extent = 
        absAxes[0] * he.x + 
        absAxes[1] * he.y + 
        absAxes[2] * he.z;
        
    Vec3 qMin = boxCenterLocal - extent;
    Vec3 qMax = boxCenterLocal + extent;

    std::vector<uint32_t> triIds;
    triIds.reserve(256);
    mesh.queryAabb(qMin, qMax, triIds, 2048);
    if (triIds.empty()) return false;

    Vec3 boxCenterM = meshBody->orientation.rotateInv(boxBody->position - meshBody->position);
    Vec3 boxAxesW[3] = {
        boxBody->orientation.rotate({1.0f, 0.0f, 0.0f}),
        boxBody->orientation.rotate({0.0f, 1.0f, 0.0f}),
        boxBody->orientation.rotate({0.0f, 0.0f, 1.0f}),
    };
    Vec3 boxAxesM[3] = {
        meshBody->orientation.rotateInv(boxAxesW[0]),
        meshBody->orientation.rotateInv(boxAxesW[1]),
        meshBody->orientation.rotateInv(boxAxesW[2]),
    };

    Vec3 boxVertsM2[8];
    for (int ix = 0; ix <= 1; ++ix) {
        float sx = ix ? 1.0f : -1.0f;
        for (int iy = 0; iy <= 1; ++iy) {
            float sy = iy ? 1.0f : -1.0f;
            for (int iz = 0; iz <= 1; ++iz) {
                float sz = iz ? 1.0f : -1.0f;
                int bi = (ix << 2) | (iy << 1) | iz;
                boxVertsM2[bi] = boxCenterM +
                    boxAxesM[0] * (he.x * sx) +
                    boxAxesM[1] * (he.y * sy) +
                    boxAxesM[2] * (he.z * sz);
            }
        }
    }

    static const int boxEdges[12][2] = {
        {0,4},{1,5},{2,6},{3,7},
        {0,2},{1,3},{4,6},{5,7},
        {0,1},{2,3},{4,5},{6,7},
    };

    auto closestPointOnObbToPoint = [&](const Vec3& p) {
        Vec3 d = p - boxCenterM;
        float x = Vec3::dot(d, boxAxesM[0]);
        float y = Vec3::dot(d, boxAxesM[1]);
        float z = Vec3::dot(d, boxAxesM[2]);
        x = clampf(x, -he.x, he.x);
        y = clampf(y, -he.y, he.y);
        z = clampf(z, -he.z, he.z);
        return boxCenterM + boxAxesM[0] * x + boxAxesM[1] * y + boxAxesM[2] * z;
    };

    auto triObbClosest = [&](const Vec3& a, const Vec3& b, const Vec3& c, Vec3& outTri, Vec3& outBox) {
        float best = 1e30f;
        Vec3 bestTri{0.0f, 0.0f, 0.0f};
        Vec3 bestBox{0.0f, 0.0f, 0.0f};

        const Vec3 triV[3] = {a, b, c};
        for (int i = 0; i < 3; ++i) {
            Vec3 bp = closestPointOnObbToPoint(triV[i]);
            float d2 = (triV[i] - bp).lengthSq();
            if (d2 < best) { best = d2; bestTri = triV[i]; bestBox = bp; if (best <= 1e-12f) break; }
        }
        if (best > 1e-12f) {
            for (int i = 0; i < 8; ++i) {
                Vec3 tp = closestPtPointTriangle(boxVertsM2[i], a, b, c);
                float d2 = (boxVertsM2[i] - tp).lengthSq();
                if (d2 < best) { best = d2; bestTri = tp; bestBox = boxVertsM2[i]; if (best <= 1e-12f) break; }
            }
        }
        if (best > 1e-12f) {
            const Vec3 e0[3] = {a, b, c};
            const Vec3 e1[3] = {b, c, a};
            for (int te = 0; te < 3; ++te) {
                for (int be = 0; be < 12; ++be) {
                    float s = 0.0f, t = 0.0f;
                    Vec3 c1, c2;
                    float d2 = closestPtSegmentSegment(e0[te], e1[te], boxVertsM2[boxEdges[be][0]], boxVertsM2[boxEdges[be][1]], s, t, c1, c2);
                    if (d2 < best) { best = d2; bestTri = c1; bestBox = c2; if (best <= 1e-12f) break; }
                }
                if (best <= 1e-12f) break;
            }
        }

        outTri = bestTri;
        outBox = bestBox;
        return best;
    };

    const float distTolSq = 1e-4f;

    struct TriCandidate {
        uint32_t tid;
        Vec3 a, b, c;
        Vec3 nLocal;
        float pen;
        float support;
        Vec3 closestTri;
        Vec3 closestBox;
    };
    std::vector<TriCandidate> candidates;
    candidates.reserve(32);

    for (uint32_t tid : triIds) {
        if (tid >= mesh.tris.size()) continue;
        const TriangleMeshTri& t = mesh.tris[tid];
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];

        Vec3 n = Vec3::cross(b - a, c - a);
        float nLenSq = n.lengthSq();
        if (nLenSq < 1e-12f) continue;
        
        Vec3 nLocal = n / sqrtf(nLenSq);
        float distCenter = Vec3::dot(boxCenterM - a, nLocal);

        if (distCenter < 0.0f) {
            Vec3 pPlane = boxCenterM - nLocal * distCenter;
            
            if (!pointInTri(pPlane, a, b, c)) { continue; }
        }

        float rOnN =
            he.x * fabsf(Vec3::dot(nLocal, boxAxesM[0])) +
            he.y * fabsf(Vec3::dot(nLocal, boxAxesM[1])) +
            he.z * fabsf(Vec3::dot(nLocal, boxAxesM[2]));

        if (distCenter >= rOnN) continue;

        float pen = (rOnN - distCenter);
        if (pen < 1e-5f) pen = 1e-5f;

        Vec3 closestTri, closestBox;
        float closestD2 = triObbClosest(a, b, c, closestTri, closestBox);
        if (closestD2 > distTolSq) continue;

        Vec3 nWorld = meshBody->orientation.rotate(nLocal);
        float support = std::max(0.0f, nWorld.y);

        TriCandidate cand;
        cand.tid = tid;
        cand.a = a; cand.b = b; cand.c = c;
        cand.nLocal = nLocal;
        cand.pen = pen;
        cand.support = support;
        cand.closestTri = closestTri;
        cand.closestBox = closestBox;
        candidates.push_back(cand);
    }

    if (candidates.empty()) return false;

    const TriCandidate* best = nullptr;
    for (const TriCandidate& c : candidates) {
        if (!best || c.pen > best->pen) {
            best = &c;
        }
    }

    if (!best) return false;

    const float minSupport = 0.05f;
    const float upwardPenTolerance = 0.001f;
    const float coplanarAngleCos = 0.9999f;
    const float coplanarSupportDiff = 0.1f;

    const TriCandidate* bestUpward = nullptr;
    for (const TriCandidate& c : candidates) {
        if (c.support >= minSupport) {
            if (!bestUpward) {
                bestUpward = &c;
            } else {
                float penDiff = c.pen - bestUpward->pen;
                if (penDiff > upwardPenTolerance || (std::fabs(penDiff) <= upwardPenTolerance && c.support > bestUpward->support)) {
                    bestUpward = &c;
                }
            }
        }
    }

    if (bestUpward) {
        float upwardScore = bestUpward->pen + bestUpward->support * 0.01f;
        float bestScore = best->pen + best->support * 0.01f;
        if (upwardScore >= bestScore - 0.001f) {
            best = bestUpward;
        }
    }

    Vec3 bestNLocal = best->nLocal;
    if (best->support >= minSupport) {
        Vec3 normalSum = bestNLocal;
        int normalCount = 1;
        
        for (const TriCandidate& c : candidates) {
            if (&c == best) continue;
            if (c.support < minSupport) continue;
            
            float normalDot = Vec3::dot(c.nLocal, bestNLocal);
            float supportDiff = std::fabs(c.support - best->support);
            if (normalDot >= coplanarAngleCos && supportDiff < coplanarSupportDiff && std::fabs(c.pen - best->pen) < upwardPenTolerance) {
                normalSum = normalSum + c.nLocal;
                normalCount++;
            }
        }
        
        if (normalCount > 1) {
            bestNLocal = normalSum / (float)normalCount;
            float len = bestNLocal.length();
            if (len > 1e-6f) {
                bestNLocal = bestNLocal / len;
            }
        }
    }
    Vec3 bestTriA = best->a;
    Vec3 bestTriB = best->b;
    Vec3 bestTriC = best->c;
    Vec3 bestClosestTri = best->closestTri;
    Vec3 bestClosestBox = best->closestBox;

    Vec3 vertsW[8];
    Vec3 vertsM[8];
    int idx = 0;
    for (int dx = -1; dx <= 1; dx += 2) {
        for (int dy = -1; dy <= 1; dy += 2) {
            for (int dz = -1; dz <= 1; dz += 2) {
                Vec3 local = {he.x * dx, he.y * dy, he.z * dz};
                Vec3 wv = boxBody->orientation.rotate(local) + boxBody->position;
                vertsW[idx] = wv;
                vertsM[idx] = meshBody->orientation.rotateInv(wv - meshBody->position);
                ++idx;
            }
        }
    }

    struct VHit { float dist = 0.0f; int i = -1; };
    VHit bestV[4];
    for (int i = 0; i < 4; ++i) { bestV[i].dist = 0.0f; bestV[i].i = -1; }

    auto insertIfBetter = [&](float dist, int vi) {
        if (dist >= 0.0f) return;
        for (int k = 0; k < 4; ++k) {
            if (bestV[k].i == vi) return;
        }
        int slot = -1;
        for (int k = 0; k < 4; ++k) {
            if (bestV[k].i < 0) { slot = k; break; }
        }
        if (slot < 0) {
            float worstDist = bestV[0].dist;
            int worstIdx = 0;
            for (int k = 1; k < 4; ++k) {
                if (bestV[k].dist > worstDist) { worstDist = bestV[k].dist; worstIdx = k; }
            }
            if (dist >= worstDist) return;
            slot = worstIdx;
        }
        bestV[slot].dist = dist;
        bestV[slot].i = vi;
    };

    for (int i = 0; i < 8; ++i) {
        float dist = Vec3::dot(vertsM[i] - bestTriA, bestNLocal);
        insertIfBetter(dist, i);
    }

    Vec3 nWorld = meshBody->orientation.rotate(bestNLocal);
    m.a = const_cast<RigidBody*>(meshBody);
    m.b = const_cast<RigidBody*>(boxBody);
    m.normal = nWorld;
    m.restitution = (meshBody->restitution + boxBody->restitution) * 0.5f;
    m.friction = (meshBody->friction + boxBody->friction) * 0.5f;
    m.frictionTwist = m.friction * 0.35f;
    m.patchR = 0.5f * (characteristicContactRadius(meshBody) + characteristicContactRadius(boxBody));
    m.count = 0;

    for (int k = 0; k < 4 && m.count < 4; ++k) {
        if (bestV[k].i < 0) continue;
        int vi = bestV[k].i;
        float dist = bestV[k].dist;
        float pen = -dist;
        if (pen < 1e-5f) pen = 1e-5f;

        Vec3 vM = vertsM[vi];
        Vec3 pPlane = vM - bestNLocal * dist;
        Vec3 triPtM = pointInTri(pPlane, bestTriA, bestTriB, bestTriC) ? pPlane : closestPtPointTriangle(pPlane, bestTriA, bestTriB, bestTriC);

        Vec3 pTriWorld = meshBody->orientation.rotate(triPtM) + meshBody->position;
        Vec3 pBoxWorld = vertsW[vi];
        Vec3 p = (pTriWorld + pBoxWorld) * 0.5f;

        m.points[m.count].pointWorld = p;
        m.points[m.count].penetration = pen;
        m.points[m.count].targetNormalVelocity = 0.0f;
        ++m.count;
    }

    if (m.count == 0) {
        Vec3 pTriWorld = meshBody->orientation.rotate(bestClosestTri) + meshBody->position;
        Vec3 pBoxWorld = meshBody->orientation.rotate(bestClosestBox) + meshBody->position;
        Vec3 p = (pTriWorld + pBoxWorld) * 0.5f;
        m.count = 1;
        m.points[0].pointWorld = p;
        m.points[0].penetration = best->pen;
        m.points[0].targetNormalVelocity = 0.0f;
    }
    return true;
}