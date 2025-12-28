#pragma once
#include "../math/Vec3.h"
#include "RigidBody.h"
#include <vector>
#include <algorithm>

struct BroadPhasePair {
    RigidBody* a;
    RigidBody* b;
};

class BroadPhase {
public:
    struct Entry {
        float minX, maxX;
        float minY, maxY;
        float minZ, maxZ;
        RigidBody* body;
    };

    std::vector<Entry> entries;
    std::vector<int> activeList;
    std::vector<BroadPhasePair> pairs;

    void build(const std::vector<RigidBody*>& bodies);
    
    static void computeAabb(const RigidBody* b, Vec3& outMin, Vec3& outMax);
};
