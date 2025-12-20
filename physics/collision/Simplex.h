#pragma once

#include "../../math/Vec3.h"

struct Simplex {
    Vec3 points[4];
    int size = 0;

    // Insert newest point at the front (index 0)
    void pushFront(const Vec3& p) {
        for (int i = size; i > 0; --i) {
            points[i] = points[i - 1];
        }
        points[0] = p;
        size++;
    }
};
