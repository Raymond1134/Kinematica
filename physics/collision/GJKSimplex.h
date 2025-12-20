#pragma once

#include "../../math/Vec3.h"
#include "Simplex.h"

// Handle line simplex (2 points)
inline bool handleLine(Simplex& simplex, Vec3& direction) {
    // A is newest point
    Vec3 A = simplex.points[0];
    Vec3 B = simplex.points[1];

    Vec3 AB = B - A;
    Vec3 AO = -A;

    if (Vec3::dot(AB, AO) > 0.0f) {
        direction = Vec3::cross(
            Vec3::cross(AB, AO),
            AB
        );
    } else {
        simplex.points[0] = A;
        simplex.size = 1;
        direction = AO;
    }

    return false;
}

// Handle triangle simplex (3 points)
inline bool handleTriangle(Simplex& simplex, Vec3& direction) {
    Vec3 A = simplex.points[0];
    Vec3 B = simplex.points[1];
    Vec3 C = simplex.points[2];

    Vec3 AO = -A;
    Vec3 AB = B - A;
    Vec3 AC = C - A;
    Vec3 ABC = Vec3::cross(AB, AC);

    Vec3 ABperp = Vec3::cross(ABC, AB);
    if (Vec3::dot(ABperp, AO) > 0.0f) {
        simplex.points[0] = A;
        simplex.points[1] = B;
        simplex.size = 2;
        return handleLine(simplex, direction);
    }

    Vec3 ACperp = Vec3::cross(AC, ABC);
    if (Vec3::dot(ACperp, AO) > 0.0f) {
        simplex.points[0] = A;
        simplex.points[1] = C;
        simplex.size = 2;
        return handleLine(simplex, direction);
    }

    if (Vec3::dot(ABC, AO) > 0.0f) {
        direction = ABC;
    } else {
        simplex.points[1] = C;
        simplex.points[2] = B;
        direction = -ABC;
    }

    return false;
}

// Handle tetrahedron simplex (4 points)
inline bool handleTetrahedron(Simplex& simplex, Vec3& direction) {
    // A is newest point
    Vec3 A = simplex.points[0];
    Vec3 B = simplex.points[1];
    Vec3 C = simplex.points[2];
    Vec3 D = simplex.points[3];

    Vec3 AO = -A;

    Vec3 ABC = Vec3::cross(B - A, C - A);
    Vec3 ACD = Vec3::cross(C - A, D - A);
    Vec3 ADB = Vec3::cross(D - A, B - A);

    if (Vec3::dot(ABC, AO) > 0.0f) {
        simplex.points[0] = A;
        simplex.points[1] = B;
        simplex.points[2] = C;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    if (Vec3::dot(ACD, AO) > 0.0f) {
        simplex.points[0] = A;
        simplex.points[1] = C;
        simplex.points[2] = D;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    if (Vec3::dot(ADB, AO) > 0.0f) {
        simplex.points[0] = A;
        simplex.points[1] = D;
        simplex.points[2] = B;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    return true;
}