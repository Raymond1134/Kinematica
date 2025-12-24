#pragma once

#include "../../math/Vec3.h"
#include "Simplex.h"

// Handle line simplex (2 points)
inline bool handleLine(Simplex& simplex, Vec3& direction) {
    // A is newest point
    SupportPoint A = simplex.verts[0];
    SupportPoint B = simplex.verts[1];
    const Vec3 a = A.p;
    const Vec3 b = B.p;
    const Vec3 ab = b - a;
    const float abLenSq = ab.lengthSq();

    if (abLenSq < 1e-12f) {
        simplex.verts[0] = A;
        simplex.size = 1;
        direction = -a;
        return false;
    }

    float t = Vec3::dot(-a, ab) / abLenSq;
    if (t <= 0.0f) {
        simplex.verts[0] = A;
        simplex.size = 1;
        direction = -a;
        return false;
    }
    if (t >= 1.0f) {
        simplex.verts[0] = B;
        simplex.size = 1;
        direction = -b;
        return false;
    }

    simplex.verts[0] = A;
    simplex.verts[1] = B;
    simplex.size = 2;
    const Vec3 closest = a + ab * t;
    direction = -closest;
    return false;
}

// Handle triangle simplex (3 points)
inline bool handleTriangle(Simplex& simplex, Vec3& direction) {
    SupportPoint A = simplex.verts[0];
    SupportPoint B = simplex.verts[1];
    SupportPoint C = simplex.verts[2];

    const Vec3 a = A.p;
    const Vec3 b = B.p;
    const Vec3 c = C.p;
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;

    const Vec3 ap = -a;
    const Vec3 bp = -b;
    const Vec3 cp = -c;

    const float d1 = Vec3::dot(ab, ap);
    const float d2 = Vec3::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        simplex.verts[0] = A;
        simplex.size = 1;
        direction = ap;
        return false;
    }

    const Vec3 bp2 = -b;
    const float d3 = Vec3::dot(ab, bp2);
    const float d4 = Vec3::dot(ac, bp2);
    if (d3 >= 0.0f && d4 <= d3) {
        simplex.verts[0] = B;
        simplex.size = 1;
        direction = bp;
        return false;
    }

    const float d5 = Vec3::dot(ab, cp);
    const float d6 = Vec3::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        simplex.verts[0] = C;
        simplex.size = 1;
        direction = cp;
        return false;
    }

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = (d1 - d3) > 1e-12f ? (d1 / (d1 - d3)) : 0.0f;
        simplex.verts[0] = A;
        simplex.verts[1] = B;
        simplex.size = 2;
        const Vec3 closest = a + ab * v;
        direction = -closest;
        return false;
    }

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = (d2 - d6) > 1e-12f ? (d2 / (d2 - d6)) : 0.0f;
        simplex.verts[0] = A;
        simplex.verts[1] = C;
        simplex.size = 2;
        const Vec3 closest = a + ac * w;
        direction = -closest;
        return false;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float denom = (d4 - d3) + (d5 - d6);
        const float w = (denom > 1e-12f) ? ((d4 - d3) / denom) : 0.0f;
        const Vec3 bc = c - b;
        simplex.verts[0] = B;
        simplex.verts[1] = C;
        simplex.size = 2;
        const Vec3 closest = b + bc * w;
        direction = -closest;
        return false;
    }

    const float denom = (va + vb + vc);
    const float invDenom = (fabsf(denom) > 1e-12f) ? (1.0f / denom) : 0.0f;
    const float v = vb * invDenom;
    const float w = vc * invDenom;
    const Vec3 closest = a + ab * v + ac * w;
    simplex.verts[0] = A;
    simplex.verts[1] = B;
    simplex.verts[2] = C;
    simplex.size = 3;
    direction = -closest;
    return false;
}

// Handle tetrahedron simplex (4 points)
inline bool handleTetrahedron(Simplex& simplex, Vec3& direction) {
    SupportPoint A = simplex.verts[0];
    SupportPoint B = simplex.verts[1];
    SupportPoint C = simplex.verts[2];
    SupportPoint D = simplex.verts[3];

    const Vec3 a = A.p;
    const Vec3 b = B.p;
    const Vec3 c = C.p;
    const Vec3 d = D.p;
    auto faceOutside = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& opp, Vec3& outN) -> bool {
        Vec3 n = Vec3::cross(p1 - p0, p2 - p0);
        if (n.lengthSq() < 1e-12f) return false;
        if (Vec3::dot(n, opp - p0) > 0.0f) n = -n;
        outN = n;
        return Vec3::dot(n, -p0) > 1e-8f;
    };

    Vec3 n;

    if (faceOutside(a, b, c, d, n)) {
        simplex.verts[0] = A; simplex.verts[1] = B; simplex.verts[2] = C;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }
    if (faceOutside(a, c, d, b, n)) {
        simplex.verts[0] = A; simplex.verts[1] = C; simplex.verts[2] = D;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }
    if (faceOutside(a, d, b, c, n)) {
        simplex.verts[0] = A; simplex.verts[1] = D; simplex.verts[2] = B;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    if (faceOutside(b, c, d, a, n)) {
        simplex.verts[0] = B; simplex.verts[1] = C; simplex.verts[2] = D;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    return true;
}