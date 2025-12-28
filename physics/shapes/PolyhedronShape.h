#pragma once

#include "../../math/Vec3.h"
#include <vector>
#include <cmath>
#include <limits>

struct PolyhedronShape {
    struct Tri { int a, b, c; };
    struct Edge { int a, b; };
    std::vector<Vec3> verts;
    std::vector<Tri> tris;
    std::vector<Edge> edges;
    float boundRadius = 0.0f;

    PolyhedronShape() = default;

    PolyhedronShape(const std::vector<Vec3>& v) : verts(v) { computeBound(); buildRenderCache(); }

private:
    void computeBound() {
        float r2 = 0.0f;
        for (const Vec3& p : verts) { r2 = std::max(r2, p.lengthSq()); }
        boundRadius = (r2 > 0.0f) ? std::sqrt(r2) : 0.0f;
    }

    static Vec3 faceNormal(const Vec3& a, const Vec3& b, const Vec3& c) { return Vec3::cross(b - a, c - a); }

    static bool pointOutsideFace(const Vec3& p, const Vec3& a, const Vec3& normalUnit, float epsDist) {
        return Vec3::dot(normalUnit, p - a) > epsDist;
    }

    void addEdgeUnique(int a, int b) {
        if (a == b) return;
        if (a > b) std::swap(a, b);
        for (const auto& e : edges) { if (e.a == a && e.b == b) return; }
        edges.push_back({a, b});
    }

public:
    float computeVolume() const {
        if (tris.empty() || verts.empty()) return 0.0f;
        double sum = 0.0;
        for (const auto& t : tris) {
            const Vec3& a = verts[t.a];
            const Vec3& b = verts[t.b];
            const Vec3& c = verts[t.c];
            double v6 = (double)Vec3::dot(a, Vec3::cross(b, c));
            sum += std::fabs(v6);
        }
        return (float)(sum / 6.0);
    }

    void buildRenderCache() {
        tris.clear();
        edges.clear();

        if (verts.size() < 4) return;

        const int n = (int)verts.size();

        int i0 = 0, i1 = -1, i2 = -1, i3 = -1;

        float maxD = 0.0f;
        for (int i = 1; i < n; ++i) {
            float d = (verts[i] - verts[i0]).lengthSq();
            if (d > maxD) {
                maxD = d;
                i1 = i;
            }
        }
        if (i1 < 0) return;

        maxD = 0.0f;
        Vec3 ab = verts[i1] - verts[i0];
        for (int i = 0; i < n; ++i) {
            Vec3 ap = verts[i] - verts[i0];
            Vec3 c = Vec3::cross(ab, ap);
            float d = c.lengthSq();
            if (d > maxD) {
                maxD = d;
                i2 = i;
            }
        }
        if (i2 < 0) return;

        Vec3 nrm = faceNormal(verts[i0], verts[i1], verts[i2]);
        maxD = 0.0f;
        for (int i = 0; i < n; ++i) {
            float d = std::fabs(Vec3::dot(nrm, verts[i] - verts[i0]));
            if (d > maxD) {
                maxD = d;
                i3 = i;
            }
        }
        if (i3 < 0) return;

        tris.push_back({i0, i1, i2});
        tris.push_back({i0, i3, i1});
        tris.push_back({i0, i2, i3});
        tris.push_back({i1, i3, i2});

        for (int p = 0; p < n; ++p) {
            if (p == i0 || p == i1 || p == i2 || p == i3) continue;

            std::vector<int> visible;
            for (int t = 0; t < (int)tris.size(); ++t) {
                const Tri& f = tris[t];
                Vec3 normal = faceNormal(verts[f.a], verts[f.b], verts[f.c]);
                float n2 = normal.lengthSq();
                if (n2 <= 1e-18f) continue;
                Vec3 nUnit = normal * (1.0f / std::sqrt(n2));
                const float epsDist = 1e-5f * std::max(1.0f, boundRadius);
                if (pointOutsideFace(verts[p], verts[f.a], nUnit, epsDist)) {
                    visible.push_back(t);
                }
            }

            if (visible.empty()) continue;

            std::vector<Edge> boundary;
            auto addBoundary = [&](int a, int b) {
                for (auto it = boundary.begin(); it != boundary.end(); ++it) {
                    if (it->a == b && it->b == a) {
                        boundary.erase(it);
                        return;
                    }
                }
                boundary.push_back({a, b});
            };

            for (int idx : visible) {
                const Tri& f = tris[idx];
                addBoundary(f.a, f.b);
                addBoundary(f.b, f.c);
                addBoundary(f.c, f.a);
            }

            for (int i = (int)visible.size() - 1; i >= 0; --i) { tris.erase(tris.begin() + visible[i]); }
            for (const Edge& e : boundary) { tris.push_back({e.a, e.b, p}); }
        }

        Vec3 center = {0.0f, 0.0f, 0.0f};
        for (const Vec3& v : verts) center += v;
        center = center * (1.0f / (float)verts.size());
        for (Tri& t : tris) {
            const Vec3& a = verts[t.a];
            const Vec3& b = verts[t.b];
            const Vec3& c = verts[t.c];
            Vec3 nrm = faceNormal(a, b, c);
            Vec3 triCenter = (a + b + c) * (1.0f / 3.0f);
            if (Vec3::dot(nrm, triCenter - center) < 0.0f) {
                std::swap(t.b, t.c);
            }
        }

        struct EdgeInfo {
            int a, b;
            int t1 = -1;
            int t2 = -1;
        };
        std::vector<EdgeInfo> edgeInfos;
        edgeInfos.reserve(tris.size() * 3);

        auto addEdgeInfo = [&](int a, int b, int tIdx) {
            if (a > b) std::swap(a, b);
            for (auto& ei : edgeInfos) {
                if (ei.a == a && ei.b == b) {
                    if (ei.t1 == -1) ei.t1 = tIdx;
                    else ei.t2 = tIdx;
                    return;
                }
            }
            edgeInfos.push_back({a, b, tIdx, -1});
        };

        for (int i = 0; i < (int)tris.size(); ++i) {
            addEdgeInfo(tris[i].a, tris[i].b, i);
            addEdgeInfo(tris[i].b, tris[i].c, i);
            addEdgeInfo(tris[i].c, tris[i].a, i);
        }

        edges.clear();
        for (const auto& ei : edgeInfos) {
            if (ei.t1 != -1 && ei.t2 != -1) {
                Vec3 n1 = faceNormal(verts[tris[ei.t1].a], verts[tris[ei.t1].b], verts[tris[ei.t1].c]).normalized();
                Vec3 n2 = faceNormal(verts[tris[ei.t2].a], verts[tris[ei.t2].b], verts[tris[ei.t2].c]).normalized();
                if (Vec3::dot(n1, n2) < 0.99f) {
                    edges.push_back({ei.a, ei.b});
                }
            } else {
                edges.push_back({ei.a, ei.b});
            }
        }
    }
};
