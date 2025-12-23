#include "TriangleMesh.h"

static TriangleMeshAabb triAabb(const Vec3& a, const Vec3& b, const Vec3& c) {
    TriangleMeshAabb out = aabbEmpty();
    aabbExpand(out, a);
    aabbExpand(out, b);
    aabbExpand(out, c);
    return out;
}

static Vec3 triCentroid(const Vec3& a, const Vec3& b, const Vec3& c) { return (a + b + c) * (1.0f / 3.0f); }

static int longestAxis(const TriangleMeshAabb& b) {
    Vec3 e = b.max - b.min;
    if (e.x >= e.y && e.x >= e.z) return 0;
    if (e.y >= e.x && e.y >= e.z) return 1;
    return 2;
}

int TriangleMesh::buildNode(std::vector<uint32_t>& triIds, uint32_t begin, uint32_t end, int leafTriCount,
                            const std::vector<TriangleMeshAabb>& triAabbs, const std::vector<Vec3>& triCentroids) {
    TriangleMeshBvhNode node;
    node.bounds = aabbEmpty();

    for (uint32_t i = begin; i < end; ++i) {
        uint32_t id = triIds[i];
        aabbMerge(node.bounds, triAabbs[id]);
    }

    const uint32_t count = end - begin;
    if (count <= (uint32_t)std::max(1, leafTriCount)) {
        node.start = (uint32_t)triOrder.size();
        node.count = count;
        for (uint32_t i = begin; i < end; ++i) triOrder.push_back(triIds[i]);
        int idx = (int)nodes.size();
        nodes.push_back(node);
        return idx;
    }

    TriangleMeshAabb centroidBounds = aabbEmpty();
    for (uint32_t i = begin; i < end; ++i) {
        aabbExpand(centroidBounds, triCentroids[triIds[i]]);
    }

    int axis = longestAxis(centroidBounds);
    float split = 0.5f * ((axis == 0 ? centroidBounds.min.x : (axis == 1 ? centroidBounds.min.y : centroidBounds.min.z)) +
                          (axis == 0 ? centroidBounds.max.x : (axis == 1 ? centroidBounds.max.y : centroidBounds.max.z)));

    auto key = [&](uint32_t id) {
        const Vec3& c = triCentroids[id];
        return (axis == 0) ? c.x : (axis == 1) ? c.y : c.z;
    };

    uint32_t mid = begin;
    for (uint32_t i = begin; i < end; ++i) {
        if (key(triIds[i]) < split) {
            std::swap(triIds[i], triIds[mid]);
            ++mid;
        }
    }

    if (mid == begin || mid == end) {
        mid = begin + count / 2;
        std::nth_element(triIds.begin() + begin, triIds.begin() + mid, triIds.begin() + end,
                         [&](uint32_t a, uint32_t b) { return key(a) < key(b); });
    }

    int idx = (int)nodes.size();
    nodes.push_back(node);

    int left = buildNode(triIds, begin, mid, leafTriCount, triAabbs, triCentroids);
    int right = buildNode(triIds, mid, end, leafTriCount, triAabbs, triCentroids);

    nodes[idx].left = left;
    nodes[idx].right = right;
    nodes[idx].bounds = aabbEmpty();
    aabbMerge(nodes[idx].bounds, nodes[left].bounds);
    aabbMerge(nodes[idx].bounds, nodes[right].bounds);

    return idx;
}

void TriangleMesh::build(const std::vector<Vec3>& verts, const std::vector<uint32_t>& indices, int leafTriCount) {
    vertices = verts;
    tris.clear();
    triOrder.clear();
    nodes.clear();

    localBounds = aabbEmpty();
    boundRadius = 0.0f;

    if (vertices.empty() || indices.size() < 3) return;

    for (const Vec3& v : vertices) aabbExpand(localBounds, v);

    const Vec3 c = localCenter();
    float r2 = 0.0f;
    for (const Vec3& v : vertices) {
        Vec3 d = v - c;
        r2 = std::max(r2, d.lengthSq());
    }
    boundRadius = std::sqrt(std::max(0.0f, r2));

    const uint32_t triCount = (uint32_t)(indices.size() / 3);
    tris.reserve(triCount);

    std::vector<TriangleMeshAabb> triAabbs;
    std::vector<Vec3> triCentroids;
    triAabbs.reserve(triCount);
    triCentroids.reserve(triCount);

    for (uint32_t i = 0; i < triCount; ++i) {
        uint32_t ia = indices[i * 3 + 0];
        uint32_t ib = indices[i * 3 + 1];
        uint32_t ic = indices[i * 3 + 2];
        if (ia >= vertices.size() || ib >= vertices.size() || ic >= vertices.size()) continue;
        TriangleMeshTri t;
        t.a = ia; t.b = ib; t.c = ic;
        tris.push_back(t);
        const Vec3& A = vertices[ia];
        const Vec3& B = vertices[ib];
        const Vec3& C = vertices[ic];
        triAabbs.push_back(triAabb(A, B, C));
        triCentroids.push_back(triCentroid(A, B, C));
    }

    if (tris.empty()) return;

    std::vector<uint32_t> triIds(tris.size());
    for (uint32_t i = 0; i < (uint32_t)triIds.size(); ++i) triIds[i] = i;

    triOrder.reserve(tris.size());
    nodes.reserve(tris.size() * 2);

    buildNode(triIds, 0, (uint32_t)triIds.size(), leafTriCount, triAabbs, triCentroids);
}

void TriangleMesh::queryAabb(const Vec3& qMin, const Vec3& qMax, std::vector<uint32_t>& outTriIds, uint32_t maxOut) const {
    outTriIds.clear();
    if (nodes.empty() || tris.empty()) return;

    TriangleMeshAabb q;
    q.min = qMin;
    q.max = qMax;

    thread_local std::vector<int> stack;
    stack.clear();
    if (stack.capacity() < 64) stack.reserve(64);
    stack.push_back(0);

    while (!stack.empty() && outTriIds.size() < maxOut) {
        int ni = stack.back();
        stack.pop_back();
        if (ni < 0 || ni >= (int)nodes.size()) continue;

        const TriangleMeshBvhNode& n = nodes[ni];
        if (!aabbOverlaps(n.bounds, q)) continue;

        if (n.isLeaf()) {
            for (uint32_t i = 0; i < n.count && outTriIds.size() < maxOut; ++i) {
                outTriIds.push_back(triOrder[n.start + i]);
            }
        } else {
            if (n.left >= 0) stack.push_back(n.left);
            if (n.right >= 0) stack.push_back(n.right);
        }
    }
}
