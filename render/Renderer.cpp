#include "Renderer.h"
#include "../physics/RigidBody.h"
#include "../math/Vec3.h"

#include <raylib.h>
#include <cmath>

bool Renderer::init(int width, int height, const char* title) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(width, height, title);
    SetTargetFPS(60);

    return IsWindowReady();
}

void Renderer::beginFrame() {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    Vector3 target = {0.0f, 0.0f, 0.0f};
    float yawRad = camYaw * (PI/180.0f);
    float pitchRad = camPitch * (PI/180.0f);
    Vector3 eye = {
        cosf(pitchRad) * cosf(yawRad) * camDistance,
        sinf(pitchRad) * camDistance,
        cosf(pitchRad) * sinf(yawRad) * camDistance
    };

    Camera3D cam{};
    cam.position = eye;
    cam.target = target;
    cam.up = {0.0f, 1.0f, 0.0f};
    cam.fovy = 60.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    BeginMode3D(cam);

    DrawGrid(20, 1.0f);
}

void Renderer::drawRigidBody(const RigidBody* body, float size) {
    if (!body) return;

    Vector3 pos = { body->position.x, body->position.y, body->position.z };
    DrawCube(pos, size, size, size, BLUE);
    DrawCubeWires(pos, size, size, size, BLACK);
}

void Renderer::endFrame() {
    EndMode3D();
    EndDrawing();

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 delta = GetMouseDelta();
        camYaw += delta.x * 0.3f;
        camPitch += -delta.y * 0.3f;
        if (camPitch > 89.0f) camPitch = 89.0f;
        if (camPitch < -89.0f) camPitch = -89.0f;
    }
    float wheel = GetMouseWheelMove();
    camDistance -= wheel * 0.5f;
    if (camDistance < 1.5f) camDistance = 1.5f;
}

void Renderer::shutdown() {
    CloseWindow();
}
