#pragma once

#include "../../math/Vec3.h"
#include "../RigidBody.h"
#include "Simplex.h"
#include "Minkowski.h"
#include "GJKSimplex.h"

inline bool gjkIntersect(
    const RigidBody& A,
    const RigidBody& B
) {
    Simplex simplex;

    Vec3 direction = {1.0f, 0.0f, 0.0f};

    Vec3 point = support(A, B, direction);
    simplex.pushFront(point);
    direction = -point;

    // Safety limits
    const int MAX_ITERATIONS = 32;

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        if (direction.lengthSq() < 1e-8f) {
            return true;
        }

        point = support(A, B, direction);

        if (Vec3::dot(point, direction) < 0.0f) {
            return false;
        }

        simplex.pushFront(point);

        if (simplex.size == 2) {
            handleLine(simplex, direction);
        }
        else if (simplex.size == 3) {
            handleTriangle(simplex, direction);
        }
        else if (simplex.size == 4) {
            if (handleTetrahedron(simplex, direction)) {
                return true;
            }
        }
    }

    return true;
}