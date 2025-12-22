#pragma once

#include "../../math/Vec3.h"

#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>

struct TriangleMeshAabb {
    Vec3 min;
    Vec3 max;
};

inline TriangleMeshAabb aabbEmpty() {
    TriangleMeshAabb a;
    a.min = {+1e30f, +1e30f, +1e30f};
    a.max = {-1e30f, -1e30f, -1e30f};
    return a;
}

inline void aabbExpand(TriangleMeshAabb& a, const Vec3& p) {
    a.min.x = std::min(a.min.x, p.x);
    a.min.y = std::min(a.min.y, p.y);
    a.min.z = std::min(a.min.z, p.z);
    a.max.x = std::max(a.max.x, p.x);
    a.max.y = std::max(a.max.y, p.y);
    a.max.z = std::max(a.max.z, p.z);
}

inline void aabbMerge(TriangleMeshAabb& a, const TriangleMeshAabb& b) {
    aabbExpand(a, b.min);
    aabbExpand(a, b.max);
}

inline bool aabbOverlaps(const TriangleMeshAabb& a, const TriangleMeshAabb& b) {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

struct TriangleMeshTri {
    uint32_t a = 0;
    uint32_t b = 0;
    uint32_t c = 0;
};

struct TriangleMeshBvhNode {
    TriangleMeshAabb bounds;
    int left = -1;
    int right = -1;
    uint32_t start = 0;
    uint32_t count = 0;

    bool isLeaf() const { return left < 0 && right < 0; }
};

struct TriangleMesh {
    enum : uint32_t {
        CollideUpwardOnly = 1u << 0,
    };

    std::vector<Vec3> vertices;
    std::vector<TriangleMeshTri> tris;
    uint32_t flags = 0;

    TriangleMeshAabb localBounds = aabbEmpty();
    float boundRadius = 0.0f;

    std::vector<uint32_t> triOrder;
    std::vector<TriangleMeshBvhNode> nodes;

    void build(const std::vector<Vec3>& verts, const std::vector<uint32_t>& indices, int leafTriCount = 4);
    void queryAabb(const Vec3& qMin, const Vec3& qMax, std::vector<uint32_t>& outTriIds, uint32_t maxOut = 4096) const;

    Vec3 localCenter() const { return (localBounds.min + localBounds.max) * 0.5f; }
    Vec3 localHalfExtents() const { return (localBounds.max - localBounds.min) * 0.5f; }

private:
    int buildNode(std::vector<uint32_t>& triIds, uint32_t begin, uint32_t end, int leafTriCount,
                  const std::vector<TriangleMeshAabb>& triAabbs, const std::vector<Vec3>& triCentroids);
};
