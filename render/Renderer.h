#pragma once

#include "../math/Vec3.h"

struct RigidBody;

class Renderer {
public:
    bool init(int width, int height, const char* title);
    void beginFrame();
    void drawRigidBody(const RigidBody* body, float size = 0.5f);
    void endFrame();
    void shutdown();
    
    Vec3 getCameraPosition() const;
    Vec3 getCameraForward() const;

private:
    Vec3 camPos = {8.0f, 6.0f, 8.0f};
    float camYaw = 225.0f;
    float camPitch = -20.0f;
    float moveSpeed = 8.0f; // meters/sec
};
