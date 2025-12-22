#pragma once

#include "../math/Vec3.h"
#include "../physics/shapes/PolyhedronShape.h"
#include <vector>
#include <raylib.h>

struct RigidBody;

class Renderer {
public:
    bool init(int width, int height, const char* title);
    void beginFrame();
    void drawRigidBody(const RigidBody* body, float size = 0.5f);
    void drawConvex(const PolyhedronShape& poly, Color color) const;
    void end3D();
    void endFrame();
    void shutdown();
    
    Vec3 getCameraPosition() const;
    Vec3 getCameraForward() const;

private:
    bool in3D = false;
    Vec3 camPos = {8.0f, 6.0f, 8.0f};
    float camYaw = 225.0f;
    float camPitch = -20.0f;
    float moveSpeed = 8.0f; // meters/sec
};
