#pragma once

#include "../../math/Vec3.h"

#include "SupportPoint.h"

struct Simplex {
    SupportPoint verts[4];
    int size = 0;

    void pushFront(const SupportPoint& v) {
        const int newSize = (size < 4) ? (size + 1) : 4;
        for (int i = newSize - 1; i > 0; --i) {
            verts[i] = verts[i - 1];
        }
        verts[0] = v;
        size = newSize;
    }
};
