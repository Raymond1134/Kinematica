#pragma once

#include "../../math/Vec3.h"
#include <vector>

struct PolyhedronShape {
	std::vector<Vec3> verts;
	float boundRadius = 0.0f;
	PolyhedronShape() = default;
	PolyhedronShape(const std::vector<Vec3>& v) : verts(v) {
		float r2 = 0.0f;
		for (const Vec3& p : verts) {
			float d2 = p.lengthSq();
			if (d2 > r2) r2 = d2;
		}
		boundRadius = (r2 > 0.0f) ? std::sqrt(r2) : 0.0f;
	}
};
