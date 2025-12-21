#pragma once

#include "../../math/Vec3.h"
#include "Simplex.h"

// Handle line simplex (2 points)
inline bool handleLine(Simplex& simplex, Vec3& direction) {
    // A is newest point
    SupportPoint A = simplex.verts[0];
    SupportPoint B = simplex.verts[1];

    Vec3 AB = B.p - A.p;
    Vec3 AO = -A.p;

    if (Vec3::dot(AB, AO) > 0.0f) {
        direction = Vec3::cross(
            Vec3::cross(AB, AO),
            AB
        );
    } else {
        simplex.verts[0] = A;
        simplex.size = 1;
        direction = AO;
    }

    return false;
}

// Handle triangle simplex (3 points)
inline bool handleTriangle(Simplex& simplex, Vec3& direction) {
    SupportPoint A = simplex.verts[0];
    SupportPoint B = simplex.verts[1];
    SupportPoint C = simplex.verts[2];

    Vec3 AO = -A.p;
    Vec3 AB = B.p - A.p;
    Vec3 AC = C.p - A.p;
    Vec3 ABC = Vec3::cross(AB, AC);

    Vec3 ABperp = Vec3::cross(ABC, AB);
    if (Vec3::dot(ABperp, AO) > 0.0f) {
        simplex.verts[0] = A;
        simplex.verts[1] = B;
        simplex.size = 2;
        return handleLine(simplex, direction);
    }

    Vec3 ACperp = Vec3::cross(AC, ABC);
    if (Vec3::dot(ACperp, AO) > 0.0f) {
        simplex.verts[0] = A;
        simplex.verts[1] = C;
        simplex.size = 2;
        return handleLine(simplex, direction);
    }

    if (Vec3::dot(ABC, AO) > 0.0f) {
        direction = ABC;
    } else {
        simplex.verts[1] = C;
        simplex.verts[2] = B;
        direction = -ABC;
    }

    return false;
}

// Handle tetrahedron simplex (4 points)
inline bool handleTetrahedron(Simplex& simplex, Vec3& direction) {
    // A is newest point
    SupportPoint A = simplex.verts[0];
    SupportPoint B = simplex.verts[1];
    SupportPoint C = simplex.verts[2];
    SupportPoint D = simplex.verts[3];

    Vec3 AO = -A.p;

    Vec3 ABC = Vec3::cross(B.p - A.p, C.p - A.p);
    Vec3 ACD = Vec3::cross(C.p - A.p, D.p - A.p);
    Vec3 ADB = Vec3::cross(D.p - A.p, B.p - A.p);

    if (Vec3::dot(ABC, AO) > 0.0f) {
        simplex.verts[0] = A;
        simplex.verts[1] = B;
        simplex.verts[2] = C;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    if (Vec3::dot(ACD, AO) > 0.0f) {
        simplex.verts[0] = A;
        simplex.verts[1] = C;
        simplex.verts[2] = D;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    if (Vec3::dot(ADB, AO) > 0.0f) {
        simplex.verts[0] = A;
        simplex.verts[1] = D;
        simplex.verts[2] = B;
        simplex.size = 3;
        return handleTriangle(simplex, direction);
    }

    return true;
}