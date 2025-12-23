#pragma once

#include "../RigidBody.h"
#include "Minkowski.h"
#include "Simplex.h"
#include "SupportPoint.h"
#include "EPATypes.h"

#include <cmath>
#include <limits>


inline bool nearlyEqualVec3(const Vec3& a, const Vec3& b, float epsSq = 1e-10f) {
    Vec3 d = a - b;
    return d.lengthSq() <= epsSq;
}

inline void addBoundaryEdge(std::vector<EPAEdgeIdx>& edges, int a, int b) {
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        if (it->a == b && it->b == a) { edges.erase(it); return; }
    }
    edges.push_back({a, b});
}

inline EPAFaceIdx makeFaceIdx(const std::vector<SupportPoint>& verts, int a, int b, int c) {
    EPAFaceIdx f;
    f.a = a; f.b = b; f.c = c;

    const Vec3& A = verts[a].p;
    const Vec3& B = verts[b].p;
    const Vec3& C = verts[c].p;

    Vec3 ab = B - A;
    Vec3 ac = C - A;
    Vec3 n = Vec3::cross(ab, ac);
    
    if (n.lengthSq() < 1e-12f) {
        f.normal = {0.0f, 1.0f, 0.0f};
        f.distance = std::numeric_limits<float>::infinity();
        return f;
    }

    f.normal = n.normalized();
    f.distance = Vec3::dot(f.normal, A);

    if (f.distance < 0.0f) {
        f.distance = -f.distance;
        f.normal = -f.normal;
        std::swap(f.b, f.c);
    }
    return f;
}

inline void closestPointToOriginOnTriangleBarycentric(const Vec3& a, const Vec3& b, const Vec3& c, float& outU, float& outV, float& outW) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ap = -a;

    float d1 = Vec3::dot(ab, ap);
    float d2 = Vec3::dot(ac, ap);

    if (d1 <= 0.0f && d2 <= 0.0f) {
        outU = 1.0f; outV = 0.0f; outW = 0.0f; return;
    }

    Vec3 bp = -b;
    float d3 = Vec3::dot(ab, bp);
    float d4 = Vec3::dot(ac, bp);

    if (d3 >= 0.0f && d4 <= d3) {
        outU = 0.0f; outV = 1.0f; outW = 0.0f; return;
    }

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        outU = 1.0f - v; outV = v; outW = 0.0f; return;
    }

    Vec3 cp = -c;
    float d5 = Vec3::dot(ab, cp);
    float d6 = Vec3::dot(ac, cp);

    if (d6 >= 0.0f && d5 <= d6) {
        outU = 0.0f; outV = 0.0f; outW = 1.0f; return;
    }

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        outU = 1.0f - w; outV = 0.0f; outW = w; return;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        outU = 0.0f; outV = 1.0f - w; outW = w; return;
    }

    float denom = 1.0f / (va + vb + vc);
    outV = vb * denom;
    outW = vc * denom;
    outU = 1.0f - outV - outW;
}

inline void ensureTetrahedron(const RigidBody& A, const RigidBody& B, std::vector<SupportPoint>& verts) {
    auto tryAdd = [&](const Vec3& dir) -> bool {
        if (verts.size() >= 4) return true;
        if (dir.lengthSq() < 1e-12f) return false;
        SupportPoint p = supportPoint(A, B, dir);
        for (const auto& v : verts) { if (nearlyEqualVec3(p.p, v.p)) return false;}
        verts.push_back(p);
        return true;
    };

    static const Vec3 axes[] = { {1,0,0}, {0,1,0}, {0,0,1}, {-1,0,0}, {0,-1,0}, {0,0,-1} };

    int safety = 0;
    while (verts.size() < 4 && safety++ < 32) {
        if (verts.size() == 1) {
            bool added = false;
            for (const Vec3& ax : axes) {
                if (tryAdd(ax)) { added = true; break; }
            }
            if (!added) break;
            continue;
        }

        if (verts.size() == 2) {
            Vec3 ab = verts[1].p - verts[0].p;
            Vec3 dir = Vec3::cross(ab, axes[0]);
            if (dir.lengthSq() < 1e-10f) dir = Vec3::cross(ab, axes[1]);
            if (dir.lengthSq() < 1e-10f) dir = Vec3::cross(ab, axes[2]);
            if (tryAdd(dir)) continue;
            if (tryAdd(-dir)) continue;
            bool added = false;
            for (const Vec3& ax : axes) {
                if (tryAdd(ax)) { added = true; break; }
            }
            if (!added) break;
            continue;
        }

        Vec3 ab = verts[1].p - verts[0].p;
        Vec3 ac = verts[2].p - verts[0].p;
        Vec3 n = Vec3::cross(ab, ac);

        if (n.lengthSq() > 1e-10f) {
            if (tryAdd(n)) continue;
            if (tryAdd(-n)) continue;
        }

        bool added = false;
        for (const Vec3& ax : axes) {
            if (tryAdd(ax)) { added = true; break; }
        }
        if (!added) break;
    }
}

inline EPAResult epaPenetration(const RigidBody& A, const RigidBody& B, const Simplex& simplex) {
    EPAResult out;

    std::vector<SupportPoint> verts;
    verts.reserve(128);
    for (int i = 0; i < simplex.size; ++i) { verts.push_back(simplex.verts[i]); }
    if (verts.empty()) return out;
    ensureTetrahedron(A, B, verts);
    if (verts.size() < 4) { return out; }

    std::vector<EPAFaceIdx> faces;
    faces.reserve(256);
    faces.push_back(makeFaceIdx(verts, 0, 1, 2));
    faces.push_back(makeFaceIdx(verts, 0, 3, 1));
    faces.push_back(makeFaceIdx(verts, 0, 2, 3));
    faces.push_back(makeFaceIdx(verts, 1, 3, 2));

    constexpr int MAX_ITERATIONS = 64;
    constexpr float TOLERANCE = 1e-4f;
    constexpr int MAX_VERTS = 128;
    constexpr int MAX_FACES = 256;

    auto fillWitnessFromFace = [&](const EPAFaceIdx& f) {
        const SupportPoint& v0 = verts[f.a];
        const SupportPoint& v1 = verts[f.b];
        const SupportPoint& v2 = verts[f.c];
        float u = 0.0f, v = 0.0f, w = 0.0f;
        closestPointToOriginOnTriangleBarycentric(v0.p, v1.p, v2.p, u, v, w);
        out.pointAWorld = v0.aWorld * u + v1.aWorld * v + v2.aWorld * w;
        out.pointBWorld = v0.bWorld * u + v1.bWorld * v + v2.bWorld * w;
        out.hasWitness = true;
    };

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        if ((int)verts.size() >= MAX_VERTS) break;
        if ((int)faces.size() >= MAX_FACES) break;

        int closest = -1;
        float minDist = std::numeric_limits<float>::infinity();
        for (int i = 0; i < (int)faces.size(); ++i) {
            if (faces[i].distance < minDist) {
                minDist = faces[i].distance;
                closest = i;
            }
        }
        if (closest < 0) break;

        const EPAFaceIdx f = faces[closest];
        if (!std::isfinite(f.distance) || f.normal.lengthSq() < 1e-12f) break;

        SupportPoint p = supportPoint(A, B, f.normal);
        float d = Vec3::dot(f.normal, p.p);
        if (!std::isfinite(d)) break;

        if (d - f.distance < TOLERANCE) {
            out.normal = f.normal;
            out.penetration = f.distance;
            fillWitnessFromFace(f);
            return out;
        }

        int newIndex = (int)verts.size();
        verts.push_back(p);

        std::vector<EPAEdgeIdx> boundary;
        boundary.reserve(128);

        for (int i = (int)faces.size() - 1; i >= 0; --i) {
            const EPAFaceIdx& face = faces[i];
            const Vec3& a0 = verts[face.a].p;
            if (Vec3::dot(face.normal, p.p - a0) > 0.0f) {
                addBoundaryEdge(boundary, face.a, face.b);
                addBoundaryEdge(boundary, face.b, face.c);
                addBoundaryEdge(boundary, face.c, face.a);
                faces.erase(faces.begin() + i);
            }
        }

        for (const auto& e : boundary) {
            EPAFaceIdx nf = makeFaceIdx(verts, e.a, e.b, newIndex);
            if (!std::isfinite(nf.distance)) continue;
            faces.push_back(nf);
            if ((int)faces.size() >= MAX_FACES) break;
        }
    }

    int best = -1;
    float bestDist = std::numeric_limits<float>::infinity();
    for (int i = 0; i < (int)faces.size(); ++i) {
        if (faces[i].distance < bestDist) {
            bestDist = faces[i].distance;
            best = i;
        }
    }
    if (best >= 0 && std::isfinite(faces[best].distance)) {
        out.normal = faces[best].normal;
        out.penetration = faces[best].distance;
        fillWitnessFromFace(faces[best]);
    }

    return out;
}

inline EPAResult epaPenetration(const RigidBody& A, const RigidBody& B, const Simplex& simplex, EPAScratch& scratch) {
    EPAResult out;

    scratch.verts.clear();
    if ((int)scratch.verts.capacity() < 128) scratch.verts.reserve(128);
    for (int i = 0; i < simplex.size; ++i) {
        scratch.verts.push_back(simplex.verts[i]);
    }
    if (scratch.verts.empty()) return out;

    ensureTetrahedron(A, B, scratch.verts);
    if (scratch.verts.size() < 4) {
        return out;
    }

    scratch.faces.clear();
    if ((int)scratch.faces.capacity() < 256) scratch.faces.reserve(256);
    scratch.faces.push_back(makeFaceIdx(scratch.verts, 0, 1, 2));
    scratch.faces.push_back(makeFaceIdx(scratch.verts, 0, 3, 1));
    scratch.faces.push_back(makeFaceIdx(scratch.verts, 0, 2, 3));
    scratch.faces.push_back(makeFaceIdx(scratch.verts, 1, 3, 2));

    constexpr int MAX_ITERATIONS = 64;
    constexpr float TOLERANCE = 1e-4f;
    constexpr int MAX_VERTS = 128;
    constexpr int MAX_FACES = 256;

    auto fillWitnessFromFace = [&](const EPAFaceIdx& f) {
        const SupportPoint& v0 = scratch.verts[f.a];
        const SupportPoint& v1 = scratch.verts[f.b];
        const SupportPoint& v2 = scratch.verts[f.c];

        float u = 0.0f, v = 0.0f, w = 0.0f;
        closestPointToOriginOnTriangleBarycentric(v0.p, v1.p, v2.p, u, v, w);
        out.pointAWorld = v0.aWorld * u + v1.aWorld * v + v2.aWorld * w;
        out.pointBWorld = v0.bWorld * u + v1.bWorld * v + v2.bWorld * w;
        out.hasWitness = true;
    };

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        if ((int)scratch.verts.size() >= MAX_VERTS) break;
        if ((int)scratch.faces.size() >= MAX_FACES) break;

        int closest = -1;
        float minDist = std::numeric_limits<float>::infinity();
        for (int i = 0; i < (int)scratch.faces.size(); ++i) {
            if (scratch.faces[i].distance < minDist) {
                minDist = scratch.faces[i].distance;
                closest = i;
            }
        }
        if (closest < 0) break;

        const EPAFaceIdx f = scratch.faces[closest];
        if (!std::isfinite(f.distance) || f.normal.lengthSq() < 1e-12f) break;

        SupportPoint p = supportPoint(A, B, f.normal);
        float d = Vec3::dot(f.normal, p.p);
        if (!std::isfinite(d)) break;

        if (d - f.distance < TOLERANCE) {
            out.normal = f.normal;
            out.penetration = f.distance;
            fillWitnessFromFace(f);
            return out;
        }

        int newIndex = (int)scratch.verts.size();
        scratch.verts.push_back(p);

        scratch.boundary.clear();
        if ((int)scratch.boundary.capacity() < 128) scratch.boundary.reserve(128);

        for (int i = (int)scratch.faces.size() - 1; i >= 0; --i) {
            const EPAFaceIdx& face = scratch.faces[i];
            const Vec3& a0 = scratch.verts[face.a].p;
            if (Vec3::dot(face.normal, p.p - a0) > 0.0f) {
                addBoundaryEdge(scratch.boundary, face.a, face.b);
                addBoundaryEdge(scratch.boundary, face.b, face.c);
                addBoundaryEdge(scratch.boundary, face.c, face.a);
                scratch.faces.erase(scratch.faces.begin() + i);
            }
        }

        for (const auto& e : scratch.boundary) {
            EPAFaceIdx nf = makeFaceIdx(scratch.verts, e.a, e.b, newIndex);
            if (!std::isfinite(nf.distance)) continue;
            scratch.faces.push_back(nf);
            if ((int)scratch.faces.size() >= MAX_FACES) break;
        }
    }

    int best = -1;
    float bestDist = std::numeric_limits<float>::infinity();
    for (int i = 0; i < (int)scratch.faces.size(); ++i) {
        if (scratch.faces[i].distance < bestDist) {
            bestDist = scratch.faces[i].distance;
            best = i;
        }
    }
    if (best >= 0) {
        out.normal = scratch.faces[best].normal;
        out.penetration = scratch.faces[best].distance;
        fillWitnessFromFace(scratch.faces[best]);
    }
    return out;
}