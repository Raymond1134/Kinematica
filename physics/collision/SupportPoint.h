#pragma once

#include "../../math/Vec3.h"

struct SupportPoint {
    Vec3 p = {0.0f, 0.0f, 0.0f};
    Vec3 aWorld = {0.0f, 0.0f, 0.0f};
    Vec3 bWorld = {0.0f, 0.0f, 0.0f};
};
