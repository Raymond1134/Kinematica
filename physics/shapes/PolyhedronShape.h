#pragma once

#include "../../math/Vec3.h"
#include <vector>
#include <cmath>

struct PolyhedronShape {
	std::vector<Vec3> verts;

	struct Tri {
		int a, b, c;
	};
	struct Edge {
		int a, b;
	};

	// Render caches (built once in ctor). This avoids O(v^4) per-frame work.
	std::vector<Tri> tris;
	std::vector<Edge> edges;

	float boundRadius = 0.0f;
	PolyhedronShape() = default;
	PolyhedronShape(const std::vector<Vec3>& v) : verts(v) {
		float r2 = 0.0f;
		for (const Vec3& p : verts) {
			float d2 = p.lengthSq();
			if (d2 > r2) r2 = d2;
		}
		boundRadius = (r2 > 0.0f) ? std::sqrt(r2) : 0.0f;

		buildRenderCache();
	}

	void buildRenderCache() {
		tris.clear();
		edges.clear();
		if (verts.size() < 4) return;

		// Naive convex hull triangulation (same logic as the old Renderer::drawConvex),
		// but done once at shape creation.
		for (size_t i = 0; i < verts.size(); ++i) {
			for (size_t j = 0; j < verts.size(); ++j) {
				if (j == i) continue;
				for (size_t k = 0; k < verts.size(); ++k) {
					if (k == i || k == j) continue;
					Vec3 v0 = verts[i];
					Vec3 v1 = verts[j];
					Vec3 v2 = verts[k];
					Vec3 normal = Vec3::cross(v1 - v0, v2 - v0);
					if (normal.x == 0 && normal.y == 0 && normal.z == 0) continue;
					bool valid = true;
					float sign = 0.0f;
					for (size_t m = 0; m < verts.size(); ++m) {
						if (m == i || m == j || m == k) continue;
						float d = Vec3::dot(normal, verts[m] - v0);
						if (sign == 0.0f && fabsf(d) > 1e-6f) sign = d;
						if (d * sign < -1e-6f) { valid = false; break; }
					}
					if (!valid) continue;

					bool duplicate = false;
					for (const auto& f : tris) {
						if ((f.a == (int)i && f.b == (int)j && f.c == (int)k) ||
							(f.a == (int)i && f.b == (int)k && f.c == (int)j) ||
							(f.a == (int)j && f.b == (int)i && f.c == (int)k) ||
							(f.a == (int)j && f.b == (int)k && f.c == (int)i) ||
							(f.a == (int)k && f.b == (int)i && f.c == (int)j) ||
							(f.a == (int)k && f.b == (int)j && f.c == (int)i)) {
							duplicate = true;
							break;
						}
					}
					if (!duplicate) tris.push_back({(int)i, (int)j, (int)k});
				}
			}
		}

		auto addEdge = [&](int a, int b) {
			if (a == b) return;
			if (a > b) std::swap(a, b);
			for (const auto& e : edges) {
				if (e.a == a && e.b == b) return;
			}
			edges.push_back({a, b});
		};

		for (const auto& t : tris) {
			addEdge(t.a, t.b);
			addEdge(t.b, t.c);
			addEdge(t.c, t.a);
		}
	}
};
