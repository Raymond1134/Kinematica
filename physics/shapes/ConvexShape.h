#pragma once

#include "../../math/Vec3.h"

struct ConvexShape {
    virtual ~ConvexShape() = default;
    virtual Vec3 support(const Vec3& direction) const = 0;
};
