#pragma once

#include "../../math/Vec3.h"

struct BoxShape {
	Vec3 halfExtents = {0.5f, 0.5f, 0.5f};
	BoxShape() = default;
	BoxShape(const Vec3& h) : halfExtents(h) {}
};
