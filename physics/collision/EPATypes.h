#pragma once

#include "SupportPoint.h"
#include <vector>

struct EPAFaceIdx {
    int a = 0;
    int b = 0;
    int c = 0;
    Vec3 normal = {0.0f, 1.0f, 0.0f};
    float distance = 0.0f;
};

struct EPAEdgeIdx {
    int a = 0;
    int b = 0;
};

struct EPAResult {
    Vec3 normal = {0.0f, 1.0f, 0.0f};
    float penetration = 0.0f;
    Vec3 pointAWorld = {0.0f, 0.0f, 0.0f};
    Vec3 pointBWorld = {0.0f, 0.0f, 0.0f};
    bool hasWitness = false;
};

struct EPAScratch {
    std::vector<SupportPoint> verts;
    std::vector<EPAFaceIdx> faces;
    std::vector<EPAEdgeIdx> boundary;
};
