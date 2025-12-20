#pragma once

#include "../../math/Vec3.h"

struct CapsuleShape {
	float radius = 0.5f;
	float halfHeight = 0.5f;
	CapsuleShape() = default;
	CapsuleShape(float r, float hh) : radius(r), halfHeight(hh) {}
};
