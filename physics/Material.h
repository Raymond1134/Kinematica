#pragma once
#include <string>
#include <raylib.h>

struct MaterialProps {
    std::string name;
    float density = 1.0f;
    float friction = 0.5f;
    float restitution = 0.0f;

    Color color = Color{200, 200, 200, 255};
    float opacity = 1.0f;
    bool outline = false;
};
