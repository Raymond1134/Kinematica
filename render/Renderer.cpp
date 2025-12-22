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
    in3D = true;

    DrawGrid(20, 1.0f);
}

void Renderer::end3D() {
    if (!in3D) return;
    EndMode3D();
    in3D = false;
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
                if (c.polyhedron) {
                    drawConvex(*c.polyhedron, color);
                }
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

void Renderer::drawConvex(const PolyhedronShape& poly, Color color) const {
    if (poly.verts.size() < 4) return;

    Color fillColor = color;
    Color edgeColor = BLACK;

    rlPushMatrix();

    rlBegin(RL_TRIANGLES);
    rlColor4ub(fillColor.r, fillColor.g, fillColor.b, fillColor.a);
    for (const auto& t : poly.tris) {
        const Vec3& a = poly.verts[t.a];
        const Vec3& b = poly.verts[t.b];
        const Vec3& c = poly.verts[t.c];
        rlVertex3f(a.x, a.y, a.z);
        rlVertex3f(b.x, b.y, b.z);
        rlVertex3f(c.x, c.y, c.z);
        rlVertex3f(a.x, a.y, a.z);
        rlVertex3f(c.x, c.y, c.z);
        rlVertex3f(b.x, b.y, b.z);
    }
    rlEnd();

    rlBegin(RL_LINES);
    rlColor4ub(edgeColor.r, edgeColor.g, edgeColor.b, edgeColor.a);
    for (const auto& e : poly.edges) {
        const Vec3& a = poly.verts[e.a];
        const Vec3& b = poly.verts[e.b];
        rlVertex3f(a.x, a.y, a.z);
        rlVertex3f(b.x, b.y, b.z);
    }
    rlEnd();

    rlPopMatrix();
}

void Renderer::endFrame() {
    end3D();
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
