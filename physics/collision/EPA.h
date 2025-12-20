#pragma once

#include "../../math/Vec3.h"
#include "Simplex.h"
#include "../RigidBody.h"
#include "Minkowski.h"
#include <vector>
#include <limits>

struct EPAFace {
    Vec3 a, b, c;
    Vec3 normal;
    float distance;
};

struct EPAResult {
    Vec3 normal;
    float penetration;
};

inline void initializePolytope(
    const Simplex& simplex,
    std::vector<EPAFace>& faces
) {
    // Simplex must be a tetrahedron for now
    if (simplex.size != 4) return;

    const Vec3& A = simplex.points[0];
    const Vec3& B = simplex.points[1];
    const Vec3& C = simplex.points[2];
    const Vec3& D = simplex.points[3];

    faces.clear();

    auto makeFace = [](const Vec3& a, const Vec3& b, const Vec3& c) -> EPAFace {
        EPAFace f;
        f.a = a; f.b = b; f.c = c;
        Vec3 ab = b - a;
        Vec3 ac = c - a;
        f.normal = Vec3::cross(ab, ac).normalized();
        f.distance = Vec3::dot(f.normal, a);
        if (f.distance < 0.0f) {
            f.normal = -f.normal;
            f.distance = -f.distance;
            std::swap(f.b, f.c);
        }
        return f;
    };

    faces.push_back(makeFace(A, B, C));
    faces.push_back(makeFace(A, C, D));
    faces.push_back(makeFace(A, D, B));
    faces.push_back(makeFace(B, D, C));
}