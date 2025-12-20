#pragma once

#include "../../math/Vec3.h"
#include <vector>

struct PolyhedronShape {
	std::vector<Vec3> verts;
	PolyhedronShape() = default;
	PolyhedronShape(const std::vector<Vec3>& v) : verts(v) {}
};
