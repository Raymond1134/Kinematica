#pragma once

#include "../math/Vec3.h"
#include "collision/EPATypes.h"
#include "RigidBody.h"
#include "Spring.h"
#include "Constraints.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <chrono>

class PhysicsWorld {
public:
    PhysicsWorld();

    // Continuous Collision Detection (CCD). This is a foundation for later TOI-based CCD.
    bool enableCCD = true;
    int ccdMaxSubsteps = 24;
    float ccdMaxTranslationFraction = 0.18f;
    float ccdMinCharacteristicRadius = 0.04f;

    bool enableSplitImpulse = true;
    int ompMinBodiesForParallel = 24;
    int ompMinPairsForParallel = 96;

    struct PerfStats {
        float stepMs = 0.0f;
        float buildContactsMs = 0.0f;
        float solveMs = 0.0f;
        int bodies = 0;
        int awake = 0;
        int broadphasePairs = 0;
        int manifolds = 0;
    };

    PerfStats perf;

    std::vector<RigidBody*> bodies;
    float currentDt = 1.0f / 60.0f;
    Vec3 gravity = {0.0f, -9.81f, 0.0f};
    std::vector<Spring> springs;
    std::vector<BallSocketJoint> ballSocketJoints;
    std::vector<HingeJoint> hingeJoints;

    // Ground plane at y=floorY. This is intentionally a dedicated plane contact.
    float floorY = 0.0f;
    bool enableFloor = true;
    float floorFriction = 0.5f;
    float floorRestitution = 0.05f;

    // Simple global damping. Values are in 1/seconds
    float linearDampingPerSecond = 0.04f;
    float angularDampingPerSecond = 0.04f;
    float floorContactAngularDampingPerSecond = 1.00f;

    // Contact-only damping for tiny residual jitter in resting stacks.
    float contactLinearDampingPerSecond = 2.00f;
    float contactAngularDampingPerSecond = 2.50f;
    float contactDampingMaxSpeed = 0.40f;
    float contactDampingMaxAngularSpeed = 1.20f;

    // Hard deadzone for resting contacts
    float contactRestVelKill = 0.005f;
    float contactRestAngVelKill = 0.015f;

    // Extra friction-only passes for resting contacts.
    int restingFrictionExtraPasses = 6;
    float restingMaxNormalSpeed = 0.12f;
    float restingMaxBodySpeed = 0.60f;
    float restingMaxBodyAngularSpeed = 2.50f;

    struct BodyPair {
        RigidBody* a;
        RigidBody* b;
        bool operator==(const BodyPair& o) const { return a == o.a && b == o.b; }
    };
    struct BodyPairHash {
        size_t operator()(const BodyPair& k) const {
            size_t h = 17;
            h = h * 31 + (size_t)k.a;
            h = h * 31 + (size_t)k.b;
            return h;
        }
    };
    std::unordered_set<BodyPair, BodyPairHash> ignoredPairs;
    void updateIgnoredPairs();

    void addBallSocketJoint(RigidBody* a, RigidBody* b, const Vec3& anchorWorld);
    void addHingeJoint(RigidBody* a, RigidBody* b, const Vec3& anchorWorld, const Vec3& axisWorld);
    void addRigidBody(RigidBody* body);
    void step(float deltaTime);

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

        bool operator==(const ContactKey& o) const { return a == o.a && b == o.b && qx == o.qx && qy == o.qy && qz == o.qz; }
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

    static void reduceManifoldToMaxPen(ContactManifold& m, int maxCount);
    static void reduceManifoldTo2(ContactManifold& m);
    static bool isFiniteVec3(const Vec3& v);
    static void buildFrictionBasis(const Vec3& n, Vec3& t1, Vec3& t2);

    static ContactKey makeContactKey(const RigidBody* a, const RigidBody* b, const Vec3& pWorld);
    const CachedImpulse* findCachedImpulseNear(const RigidBody* a, const RigidBody* b, const Vec3& pWorld) const;
    void pruneContactCache();

    void buildContacts(std::vector<ContactManifold>& out);

    struct SapEntry {
        float minX;
        float maxX;
        float minY;
        float maxY;
        float minZ;
        float maxZ;
        RigidBody* body;
    };
    struct SapPair {
        RigidBody* a;
        RigidBody* b;
    };
    std::vector<SapEntry> sapEntries;
    std::vector<SapPair> sapPairs;
    std::vector<int> sapActive;

    std::vector<std::vector<ContactManifold>> contactLocals;
    std::vector<EPAScratch> epaLocals;

    void ensureContactLocals(int maxThreads);
    void buildSapCandidates();
    bool detectFloor(RigidBody* body, ContactManifold& m);
    void warmStartContacts(std::vector<ContactManifold>& ms);

    static float clampf(float v, float lo, float hi);
    static void getBoxAxes(const RigidBody* b, Vec3 outAxes[3]);
    static float projectBoxRadiusOnAxis(const RigidBody* b, const Vec3 axes[3], const Vec3& axisUnit);
    static int clipPolygonToPlane(const Vec3* inVerts, int inCount, Vec3* outVerts, const Vec3& n, float d);

    bool collideSphereSphere(RigidBody* a, RigidBody* b, ContactManifold& m);
    bool collideSphereBox(RigidBody* sphereBody, RigidBody* boxBody, ContactManifold& m);

    static float closestPtSegmentSegment(
        const Vec3& p1,
        const Vec3& q1,
        const Vec3& p2,
        const Vec3& q2,
        float& s,
        float& t,
        Vec3& c1,
        Vec3& c2
    );

    bool collideCapsuleSphere(RigidBody* capsuleBody, RigidBody* sphereBody, ContactManifold& m);
    bool collideCapsuleCapsule(RigidBody* a, RigidBody* b, ContactManifold& m);
    bool collideBoxBox(RigidBody* a, RigidBody* b, ContactManifold& m);
    void appendBodyBodyMeshContacts(RigidBody* a, RigidBody* b, std::vector<ContactManifold>& out);
    void appendMeshBoxManifolds(const RigidBody* meshBody, const RigidBody* boxBody, std::vector<ContactManifold>& out, int maxManifolds) const;
    void appendMeshConvexManifolds(const RigidBody* meshBody, const RigidBody* convexBody, std::vector<ContactManifold>& out, int maxManifolds) const;
    bool collideMeshSphere(const RigidBody* meshBody, const RigidBody* sphereBody, ContactManifold& m) const;
    bool collideMeshCapsule(const RigidBody* meshBody, const RigidBody* capsuleBody, ContactManifold& m) const;
    bool collideMeshBox(const RigidBody* meshBody, const RigidBody* boxBody, ContactManifold& m) const;
    static bool pointInTri(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c);
    static Vec3 closestPtPointTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c);

    void computeRestitutionTargets(std::vector<ContactManifold>& ms);
    void solveContacts(std::vector<ContactManifold>& ms, bool applyPositionCorrection);
    void solveContacts(const std::vector<ContactManifold*>& ms, bool applyPositionCorrection);
    void solveRestingFriction(std::vector<ContactManifold>& ms, int passes);
    void solveRestingFriction(const std::vector<ContactManifold*>& ms, int passes);
    void solveJoints(const std::vector<BallSocketJoint*>& joints, float dt, bool isFirst);
    void solveHingeJoints(const std::vector<HingeJoint*>& joints, float dt, bool isFirst);
    void solveIslands(std::vector<ContactManifold>& contacts);

    static RigidBody makeChildProxyBody(const RigidBody* parent, const CompoundChild& child);
    static void orientNormalForPair(ContactManifold& m, const RigidBody* a, const RigidBody* b);
    void appendBodyBodyContacts(RigidBody* a, RigidBody* b, std::vector<ContactManifold>& out, EPAScratch& epaScratch);
    bool detectBodyBodyConvex(RigidBody* a, RigidBody* b, ContactManifold& m, EPAScratch& epaScratch);
    Vec3 invInertiaWorldMul(const RigidBody* body, const Vec3& v) const;
    void applyImpulseAtPoint(RigidBody* body, const Vec3& impulse, const Vec3& pointWorld, bool wake = true);
    void applyAngularImpulse(RigidBody* body, const Vec3& angularImpulseWorld, bool wake = true);
    float characteristicContactRadius(const RigidBody* body) const;

private:
    std::vector<std::vector<ContactManifold*>> islandBuffer;
    std::vector<std::vector<BallSocketJoint*>> islandJointsBuffer;
    std::vector<std::vector<HingeJoint*>> islandHingeJointsBuffer;
    std::vector<int> islandParentBuffer;
    std::vector<int> islandRootToIdBuffer;

    void stepSubstep(float deltaTime, bool isFinalSubstep);
    int computeCcdSubsteps(float deltaTime) const;
};