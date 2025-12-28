#pragma once

#include "../math/Vec3.h"
#include <raylib.h>
#include <cmath>

struct Plane {
    Vec3 normal;
    float distance;

    void normalize() {
        float len = normal.length();
        if (len > 1e-6f) {
            float invLen = 1.0f / len;
            normal = normal * invLen;
            distance *= invLen;
        }
    }

    float distanceToPoint(const Vec3& point) const {
        return Vec3::dot(normal, point) + distance;
    }
};

class Frustum {
public:
    Plane planes[6];

    void update(const Matrix& viewProj) {
        planes[0].normal.x = viewProj.m3 + viewProj.m0;
        planes[0].normal.y = viewProj.m7 + viewProj.m4;
        planes[0].normal.z = viewProj.m11 + viewProj.m8;
        planes[0].distance = viewProj.m15 + viewProj.m12;

        planes[1].normal.x = viewProj.m3 - viewProj.m0;
        planes[1].normal.y = viewProj.m7 - viewProj.m4;
        planes[1].normal.z = viewProj.m11 - viewProj.m8;
        planes[1].distance = viewProj.m15 - viewProj.m12;

        planes[2].normal.x = viewProj.m3 + viewProj.m1;
        planes[2].normal.y = viewProj.m7 + viewProj.m5;
        planes[2].normal.z = viewProj.m11 + viewProj.m9;
        planes[2].distance = viewProj.m15 + viewProj.m13;

        planes[3].normal.x = viewProj.m3 - viewProj.m1;
        planes[3].normal.y = viewProj.m7 - viewProj.m5;
        planes[3].normal.z = viewProj.m11 - viewProj.m9;
        planes[3].distance = viewProj.m15 - viewProj.m13;

        planes[4].normal.x = viewProj.m3 + viewProj.m2;
        planes[4].normal.y = viewProj.m7 + viewProj.m6;
        planes[4].normal.z = viewProj.m11 + viewProj.m10;
        planes[4].distance = viewProj.m15 + viewProj.m14;

        planes[5].normal.x = viewProj.m3 - viewProj.m2;
        planes[5].normal.y = viewProj.m7 - viewProj.m6;
        planes[5].normal.z = viewProj.m11 - viewProj.m10;
        planes[5].distance = viewProj.m15 - viewProj.m14;

        for (int i = 0; i < 6; i++) {
            planes[i].normalize();
        }
    }

    bool containsSphere(const Vec3& center, float radius) const {
        for (int i = 0; i < 6; i++) {
            if (planes[i].distanceToPoint(center) < -radius) {
                return false;
            }
        }
        return true;
    }
};
