#pragma once

#include "../../math/Vec3.h"
#include "../RigidBody.h"
#include "Simplex.h"
#include "Minkowski.h"
#include "GJKSimplex.h"

struct GJKResult {
    bool hit = false;
    Simplex simplex;
};

inline GJKResult gjk(const RigidBody& A, const RigidBody& B) {
    GJKResult result;
    Simplex simplex;
    Vec3 direction = B.position - A.position;
    if (direction.lengthSq() < 1e-12f) direction = {1.0f, 0.0f, 0.0f};
    SupportPoint v = supportPoint(A, B, direction);

    simplex.pushFront(v);
    direction = -v.p;

    const int MAX_ITERATIONS = 48;

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        if (direction.lengthSq() < 1e-8f) {
            result.hit = true;
            result.simplex = simplex;
            return result;
        }

        v = supportPoint(A, B, direction);


        float vd = Vec3::dot(v.p, direction);
        if (vd < -1e-7f) {
            result.hit = false;
            return result;
        }

        simplex.pushFront(v);

        if (simplex.size == 2) { handleLine(simplex, direction); }
        else if (simplex.size == 3) { handleTriangle(simplex, direction); }
        else if (simplex.size == 4) {
            if (handleTetrahedron(simplex, direction)) {
                result.hit = true;
                result.simplex = simplex;
                return result;
            }
        }
    }

    result.hit = false;
    result.simplex = simplex;
    return result;
}

inline bool gjkIntersect(const RigidBody& A, const RigidBody& B) { return gjk(A, B).hit; }