#pragma once

#include "../../math/Vec3.h"
#include "../RigidBody.h"

inline Vec3 support(
    const RigidBody& A,
    const RigidBody& B,
    const Vec3& direction
) {
    Vec3 dirA_local = A.orientation.rotateInv(direction);
    Vec3 dirB_local = B.orientation.rotateInv(-direction);

    Vec3 pA_local = A.collider.support(dirA_local);
    Vec3 pB_local = B.collider.support(dirB_local);

    Vec3 pA_world = A.orientation.rotate(pA_local) + A.position;
    Vec3 pB_world = B.orientation.rotate(pB_local) + B.position;

    return pA_world - pB_world;
}