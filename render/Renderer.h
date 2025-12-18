#pragma once

struct RigidBody;

class Renderer {
public:
    bool init(int width, int height, const char* title);
    void beginFrame();
    void drawRigidBody(const RigidBody* body, float size = 0.5f);
    void endFrame();
    void shutdown();

private:
    float camYaw = 45.0f;
    float camPitch = 30.0f;
    float camDistance = 12.0f;
};
