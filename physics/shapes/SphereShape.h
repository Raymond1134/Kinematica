#pragma once

#include "../../math/Vec3.h"

struct SphereShape {
	float radius = 0.5f;
	SphereShape() = default;
	SphereShape(float r) : radius(r) {}
};
