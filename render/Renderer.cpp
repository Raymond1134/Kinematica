#include "Renderer.h"
#include "../physics/RigidBody.h"
#include "../math/Vec3.h"
#include "../math/Quat.h"

#include <raylib.h>
#include <rlgl.h>
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

    float yawRad = camYaw * (PI/180.0f);
    float pitchRad = camPitch * (PI/180.0f);

    Vec3 forward = {
        cosf(pitchRad) * cosf(yawRad),
        sinf(pitchRad),
        cosf(pitchRad) * sinf(yawRad)
    };
    forward = forward.normalized();

    Vector3 eye = {camPos.x, camPos.y, camPos.z};
    Vector3 target = {camPos.x + forward.x, camPos.y + forward.y, camPos.z + forward.z};

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
    Color color = body->isStatic ? GRAY : BLUE;
    
    Quat q = body->orientation.normalized();
    float qw = q.w;
    if (qw > 1.0f) qw = 1.0f;
    if (qw < -1.0f) qw = -1.0f;

    float rotationAngle = 2.0f * acosf(qw);
    float s = sqrtf(1.0f - qw * qw);
    Vector3 rotationAxis = {1.0f, 0.0f, 0.0f};
    if (s > 0.0001f) {
        rotationAxis = {q.x / s, q.y / s, q.z / s};
    }
    
    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    
    if (rotationAngle > 0.001f) {
        rlRotatef(rotationAngle * RAD2DEG, rotationAxis.x, rotationAxis.y, rotationAxis.z);
    }
    
    switch (body->collider.type) {
        case ColliderType::Sphere: {
            float radius = body->collider.sphere.radius;
            DrawSphere({0, 0, 0}, radius, color);
            DrawSphereWires({0, 0, 0}, radius, 16, 16, Fade(BLACK, 0.7f));
            DrawLine3D({0, 0, 0}, {radius, 0, 0}, Fade(RED, 0.7f));
            DrawLine3D({0, 0, 0}, {0, radius, 0}, Fade(GREEN, 0.7f));
            DrawLine3D({0, 0, 0}, {0, 0, radius}, Fade(BLUE, 0.7f));
            break;
        }
        case ColliderType::Box: {
            Vec3 half = body->collider.box.halfExtents;
            Vector3 boxSize = {half.x * 2.0f, half.y * 2.0f, half.z * 2.0f};
            DrawCubeV({0, 0, 0}, boxSize, color);
            Vector3 outlineSize = {boxSize.x, boxSize.y, boxSize.z};
            DrawCubeWiresV({0, 0, 0}, outlineSize, BLACK);
            break;
        }
        case ColliderType::Capsule: {
            float radius = body->collider.capsule.radius;
            float halfHeight = body->collider.capsule.halfHeight;
            DrawCapsule({0, -halfHeight, 0}, {0, halfHeight, 0}, radius, 8, 8, color);
            DrawCapsuleWires({0, -halfHeight, 0}, {0, halfHeight, 0}, radius, 16, 16, Fade(BLACK, 0.7f));
            DrawLine3D({0, -halfHeight, 0}, {0, halfHeight, 0}, Fade(RED, 0.7f));
            break;
        }
    }
    
    rlPopMatrix();
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

    float dt = GetFrameTime();
    float yawRad = camYaw * (PI/180.0f);
    float pitchRad = camPitch * (PI/180.0f);
    Vec3 forward = {
        cosf(pitchRad) * cosf(yawRad),
        sinf(pitchRad),
        cosf(pitchRad) * sinf(yawRad)
    };
    forward = forward.normalized();
    Vec3 right = Vec3::cross({0.0f, 1.0f, 0.0f}, forward).normalized();


    Vec3 move = {0.0f, 0.0f, 0.0f};
    if (IsKeyDown(KEY_W)) move += forward;
    if (IsKeyDown(KEY_S)) move -= forward;
    if (IsKeyDown(KEY_A)) move += right;
    if (IsKeyDown(KEY_D)) move -= right;
    if (IsKeyDown(KEY_SPACE)) move.y += 1.0f;
    if (IsKeyDown(KEY_LEFT_SHIFT)) move.y -= 1.0f;

    float mLenSq = move.x * move.x + move.y * move.y + move.z * move.z;
    if (mLenSq > 1e-10f) {
        float inv = 1.0f / sqrtf(mLenSq);
        move = move * inv;
        camPos += move * (moveSpeed * dt);
    }

    float wheel = GetMouseWheelMove();
    if (fabsf(wheel) > 1e-6f) {
        camPos += forward * (wheel * 1.5f);
    }
}

void Renderer::shutdown() {
    CloseWindow();
}

Vec3 Renderer::getCameraPosition() const {
    return camPos;
}

Vec3 Renderer::getCameraForward() const {
    float yawRad = camYaw * (PI/180.0f);
    float pitchRad = camPitch * (PI/180.0f);
    Vec3 forward = {
        cosf(pitchRad) * cosf(yawRad),
        sinf(pitchRad),
        cosf(pitchRad) * sinf(yawRad)
    };
    return forward.normalized();
}
