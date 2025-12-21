#include "Renderer.h"
#include "../physics/RigidBody.h"
#include "../math/Vec3.h"
#include "../math/Quat.h"
#include <vector>
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
    
    auto applyQuat = [&](const Quat& qq) {
        Quat nq = qq.normalized();
        float w = nq.w;
        if (w > 1.0f) w = 1.0f;
        if (w < -1.0f) w = -1.0f;

        float ang = 2.0f * acosf(w);
        float ss = sqrtf(1.0f - w * w);
        Vector3 axis = {1.0f, 0.0f, 0.0f};
        if (ss > 0.0001f) {
            axis = {nq.x / ss, nq.y / ss, nq.z / ss};
        }
        if (ang > 0.001f) {
            rlRotatef(ang * RAD2DEG, axis.x, axis.y, axis.z);
        }
    };

    auto drawColliderAtOrigin = [&](const Collider& c) {
        switch (c.type) {
            case ColliderType::Sphere: {
                float radius = c.sphere.radius;
                DrawSphere(Vector3{0.0f, 0.0f, 0.0f}, radius, color);
                DrawSphereWires(Vector3{0.0f, 0.0f, 0.0f}, radius, 16, 16, Fade(BLACK, 0.7f));
                DrawLine3D(Vector3{0.0f, 0.0f, 0.0f}, Vector3{radius, 0.0f, 0.0f}, Fade(RED, 0.7f));
                DrawLine3D(Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, radius, 0.0f}, Fade(GREEN, 0.7f));
                DrawLine3D(Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, radius}, Fade(BLUE, 0.7f));
                break;
            }
            case ColliderType::Box: {
                Vec3 half = c.box.halfExtents;
                Vector3 boxSize = {half.x * 2.0f, half.y * 2.0f, half.z * 2.0f};
                DrawCubeV(Vector3{0.0f, 0.0f, 0.0f}, boxSize, color);
                Vector3 outlineSize = {boxSize.x, boxSize.y, boxSize.z};
                DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, outlineSize, BLACK);
                break;
            }
            case ColliderType::Capsule: {
                float radius = c.capsule.radius;
                float halfHeight = c.capsule.halfHeight;
                DrawCapsule(Vector3{0.0f, -halfHeight, 0.0f}, Vector3{0.0f, halfHeight, 0.0f}, radius, 8, 8, color);
                DrawCapsuleWires(Vector3{0.0f, -halfHeight, 0.0f}, Vector3{0.0f, halfHeight, 0.0f}, radius, 16, 16, Fade(BLACK, 0.7f));
                DrawLine3D(Vector3{0.0f, -halfHeight, 0.0f}, Vector3{0.0f, halfHeight, 0.0f}, Fade(RED, 0.7f));
                break;
            }
            case ColliderType::Convex: {
                drawConvex(c.polyhedron.verts, color);
                break;
            }
            case ColliderType::Mesh:
            case ColliderType::Compound:
                break;
        }
    };

    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    applyQuat(body->orientation);
    
    switch (body->collider.type) {
        case ColliderType::Compound: {
            if (body->collider.compound) {
                for (const CompoundChild& child : body->collider.compound->children) {
                    rlPushMatrix();
                    rlTranslatef(child.localPosition.x, child.localPosition.y, child.localPosition.z);
                    applyQuat(child.localOrientation);
                    drawColliderAtOrigin(child.collider);
                    rlPopMatrix();
                }
            }
            break;
        }
        default:
            drawColliderAtOrigin(body->collider);
            break;
    }

    rlPopMatrix();
}

void Renderer::drawConvex(const std::vector<Vec3>& verts, Color color) const {
    if (verts.size() < 4) return;
    Color fillColor = color;
    Color edgeColor = BLACK;
    struct Face { int a, b, c; };
    std::vector<Face> faces;
    // Naive convex hull triangulation
    for (size_t i = 0; i < verts.size(); ++i) {
        for (size_t j = 0; j < verts.size(); ++j) {
            if (j == i) continue;
            for (size_t k = 0; k < verts.size(); ++k) {
                if (k == i || k == j) continue;
                Vec3 v0 = verts[i];
                Vec3 v1 = verts[j];
                Vec3 v2 = verts[k];
                Vec3 normal = Vec3::cross(v1 - v0, v2 - v0);
                if (normal.x == 0 && normal.y == 0 && normal.z == 0) continue;
                bool valid = true;
                float sign = 0.0f;
                for (size_t m = 0; m < verts.size(); ++m) {
                    if (m == i || m == j || m == k) continue;
                    float d = Vec3::dot(normal, verts[m] - v0);
                    if (sign == 0.0f && fabsf(d) > 1e-6f) sign = d;
                    if (d * sign < -1e-6f) { valid = false; break; }
                }
                if (valid) {
                    bool duplicate = false;
                    for (const auto& f : faces) {
                        if ((f.a == i && f.b == j && f.c == k) ||
                            (f.a == i && f.b == k && f.c == j) ||
                            (f.a == j && f.b == i && f.c == k) ||
                            (f.a == j && f.b == k && f.c == i) ||
                            (f.a == k && f.b == i && f.c == j) ||
                            (f.a == k && f.b == j && f.c == i)) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate) faces.push_back({(int)i, (int)j, (int)k});
                }
            }
        }
    }
    rlPushMatrix();
    rlBegin(RL_TRIANGLES);
    rlColor4ub(fillColor.r, fillColor.g, fillColor.b, fillColor.a);
    for (const auto& f : faces) {
        rlVertex3f(verts[f.a].x, verts[f.a].y, verts[f.a].z);
        rlVertex3f(verts[f.b].x, verts[f.b].y, verts[f.b].z);
        rlVertex3f(verts[f.c].x, verts[f.c].y, verts[f.c].z);
        rlVertex3f(verts[f.a].x, verts[f.a].y, verts[f.a].z);
        rlVertex3f(verts[f.c].x, verts[f.c].y, verts[f.c].z);
        rlVertex3f(verts[f.b].x, verts[f.b].y, verts[f.b].z);
    }
    rlEnd();
    rlBegin(RL_LINES);
    rlColor4ub(edgeColor.r, edgeColor.g, edgeColor.b, edgeColor.a);
    for (const auto& f : faces) {
        rlVertex3f(verts[f.a].x, verts[f.a].y, verts[f.a].z);
        rlVertex3f(verts[f.b].x, verts[f.b].y, verts[f.b].z);
        rlVertex3f(verts[f.b].x, verts[f.b].y, verts[f.b].z);
        rlVertex3f(verts[f.c].x, verts[f.c].y, verts[f.c].z);
        rlVertex3f(verts[f.c].x, verts[f.c].y, verts[f.c].z);
        rlVertex3f(verts[f.a].x, verts[f.a].y, verts[f.a].z);
    }
    rlEnd();
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
