#pragma once

#include "../../math/Vec3.h"
#include "../RigidBody.h"

#include "SupportPoint.h"

inline SupportPoint supportPoint(
    const RigidBody& A,
    const RigidBody& B,
    const Vec3& direction
) {
    Vec3 dirA_local = A.orientation.rotateInv(direction);
    Vec3 dirB_local = B.orientation.rotateInv(-direction);

    Vec3 pA_local = A.collider.support(dirA_local);
    Vec3 pB_local = B.collider.support(dirB_local);

    SupportPoint out;
    out.aWorld = A.orientation.rotate(pA_local) + A.position;
    out.bWorld = B.orientation.rotate(pB_local) + B.position;
    out.p = out.aWorld - out.bWorld;
    return out;
}

inline Vec3 support(
    const RigidBody& A,
    const RigidBody& B,
    const Vec3& direction
) {
    return supportPoint(A, B, direction).p;
}