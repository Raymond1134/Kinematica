#include "Renderer.h"
#include "../physics/RigidBody.h"
#include "../math/Vec3.h"
#include "../math/Quat.h"
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <cmath>

bool Renderer::init(int width, int height, const char* title) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(width, height, title);
    SetTargetFPS(60);

    if (!meshesInitialized) {
        cubeMesh = GenMeshCube(1.0f, 1.0f, 1.0f);
        sphereMesh = GenMeshSphere(1.0f, 16, 16);
        meshesInitialized = true;
    }

    return IsWindowReady();
}

void Renderer::beginFrame() {
    for (auto& kv : cubeInstances) kv.second.clear();
    for (auto& kv : sphereInstances) kv.second.clear();

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

    if (!cubeInstances.empty()) {
        Material mat = LoadMaterialDefault();
        for (const auto& kv : cubeInstances) {
            unsigned int cInt = kv.first;
            Color c = { (unsigned char)(cInt >> 24), (unsigned char)((cInt >> 16) & 0xFF), (unsigned char)((cInt >> 8) & 0xFF), (unsigned char)(cInt & 0xFF) };
            mat.maps[MATERIAL_MAP_DIFFUSE].color = c;
            DrawMeshInstanced(cubeMesh, mat, kv.second.data(), (int)kv.second.size());
        }
    }

    if (!sphereInstances.empty()) {
        Material mat = LoadMaterialDefault();
        for (const auto& kv : sphereInstances) {
            unsigned int cInt = kv.first;
            Color c = { (unsigned char)(cInt >> 24), (unsigned char)((cInt >> 16) & 0xFF), (unsigned char)((cInt >> 8) & 0xFF), (unsigned char)(cInt & 0xFF) };
            mat.maps[MATERIAL_MAP_DIFFUSE].color = c;
            DrawMeshInstanced(sphereMesh, mat, kv.second.data(), (int)kv.second.size());
        }
    }

    EndMode3D();
    in3D = false;
}

void Renderer::setUiBlockRect(Rectangle r) {
    uiBlockRect = r;
    hasUiBlockRect = (r.width > 0.0f && r.height > 0.0f);
}

static uint64_t edgeKey(uint32_t a, uint32_t b) {
    uint32_t lo = (a < b) ? a : b;
    uint32_t hi = (a < b) ? b : a;
    return (uint64_t(lo) << 32) | uint64_t(hi);
}

const std::vector<Renderer::Edge>& Renderer::getMeshBoundaryEdges(const TriangleMesh& mesh) const {
    for (const MeshEdgeCache& c : meshEdgeCaches) {
        if (c.mesh == &mesh) return c.boundaryEdges;
    }

    MeshEdgeCache cache;
    cache.mesh = &mesh;
    cache.boundaryEdges.clear();

    struct EdgeAccum {
        int count = 0;
        Vec3 n0 = {0.0f, 0.0f, 0.0f};
        Vec3 n1 = {0.0f, 0.0f, 0.0f};
        bool has0 = false;
        bool has1 = false;
    };

    std::unordered_map<uint64_t, EdgeAccum> acc;
    acc.reserve(mesh.tris.size() * 3);

    auto addEdge = [&](uint32_t a, uint32_t b, const Vec3& triNormal) {
        EdgeAccum& e = acc[edgeKey(a, b)];
        e.count++;
        if (!e.has0) {
            e.n0 = triNormal;
            e.has0 = true;
        } else if (!e.has1) {
            e.n1 = triNormal;
            e.has1 = true;
        }
    };

    for (const auto& t : mesh.tris) {
        if (t.a >= mesh.vertices.size() || t.b >= mesh.vertices.size() || t.c >= mesh.vertices.size()) continue;
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];
        Vec3 n = Vec3::cross(b - a, c - a).normalized();
        if (n.lengthSq() <= 1e-12f) continue;

        addEdge(t.a, t.b, n);
        addEdge(t.b, t.c, n);
        addEdge(t.c, t.a, n);
    }


    constexpr float kCosFeatureAngle = 0.985f;

    cache.boundaryEdges.reserve(acc.size());
    for (const auto& kv : acc) {
        const EdgeAccum& e = kv.second;
        bool include = false;
        if (e.count == 1) {
            include = true;
        } else if (e.count == 2) {
            if (!e.has0 || !e.has1) {
                include = true;
            } else {
                float d = Vec3::dot(e.n0, e.n1);
                include = (d < kCosFeatureAngle);
            }
        } else {
            include = true;
        }

        if (!include) continue;
        uint32_t a = uint32_t(kv.first >> 32);
        uint32_t b = uint32_t(kv.first & 0xffffffffu);
        cache.boundaryEdges.push_back({a, b});
    }

    meshEdgeCaches.push_back(std::move(cache));
    return meshEdgeCaches.back().boundaryEdges;
}

void Renderer::drawRigidBody(const RigidBody* body, const RenderStyle& style, float size) {
    if (!body) return;

    Vector3 pos = { body->position.x, body->position.y, body->position.z };
    Color color = style.color;
    bool outline = style.outline;

    const bool isTransparent = (color.a < 255);
    if (isTransparent) {
        rlDrawRenderBatchActive();
        BeginBlendMode(BLEND_ALPHA);
        rlDisableDepthMask();
    }
    
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

    auto drawTriangleMeshAtOrigin = [&](const TriangleMesh& mesh, bool allowOutline) {
        Color fill = color;
        rlBegin(RL_TRIANGLES);
        rlColor4ub(fill.r, fill.g, fill.b, fill.a);
        for (const auto& t : mesh.tris) {
            if (t.a >= mesh.vertices.size() || t.b >= mesh.vertices.size() || t.c >= mesh.vertices.size()) continue;
            const Vec3& a = mesh.vertices[t.a];
            const Vec3& b = mesh.vertices[t.b];
            const Vec3& c0 = mesh.vertices[t.c];
            rlVertex3f(a.x, a.y, a.z);
            rlVertex3f(b.x, b.y, b.z);
            rlVertex3f(c0.x, c0.y, c0.z);

            rlVertex3f(a.x, a.y, a.z);
            rlVertex3f(c0.x, c0.y, c0.z);
            rlVertex3f(b.x, b.y, b.z);
        }
        rlEnd();

        const bool doOutline = outline && allowOutline;
        if (doOutline) {
            const auto& edges = getMeshBoundaryEdges(mesh);
            rlBegin(RL_LINES);
            rlColor4ub(0, 0, 0, 255);
            for (const auto& e : edges) {
                if (e.a >= mesh.vertices.size() || e.b >= mesh.vertices.size()) continue;
                const Vec3& a = mesh.vertices[e.a];
                const Vec3& b = mesh.vertices[e.b];
                rlVertex3f(a.x, a.y, a.z);
                rlVertex3f(b.x, b.y, b.z);
            }
            rlEnd();
        }
    };

    auto drawColliderAtOrigin = [&](const Collider& c, bool allowOutline, Matrix currentTransform) {
        switch (c.type) {
            case ColliderType::Sphere: {
                float radius = c.sphere.radius;
                const bool doOutline = outline && allowOutline;
                if (doOutline || isTransparent) {
                    DrawSphere(Vector3{0.0f, 0.0f, 0.0f}, radius, color);
                    if (doOutline) DrawSphereWires(Vector3{0.0f, 0.0f, 0.0f}, radius, 16, 16, BLACK);
                } else {
                    Matrix mat = MatrixIdentity();
                    mat = MatrixMultiply(MatrixScale(radius, radius, radius), mat);
                    mat = MatrixMultiply(mat, currentTransform);
                    
                    unsigned int cInt = (color.r << 24) | (color.g << 16) | (color.b << 8) | color.a;
                    sphereInstances[cInt].push_back(mat);
                }
                break;
            }
            case ColliderType::Box: {
                Vec3 half = c.box.halfExtents;
                Vector3 boxSize = {half.x * 2.0f, half.y * 2.0f, half.z * 2.0f};
                const bool doOutline = outline && allowOutline;
                
                if (doOutline || isTransparent) {
                    DrawCubeV(Vector3{0.0f, 0.0f, 0.0f}, boxSize, color);
                    if (doOutline) {
                        Vector3 outlineSize = {boxSize.x, boxSize.y, boxSize.z};
                        DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, outlineSize, BLACK);
                    }
                } else {
                    Matrix mat = MatrixIdentity();
                    mat = MatrixMultiply(MatrixScale(boxSize.x, boxSize.y, boxSize.z), mat);
                    mat = MatrixMultiply(mat, currentTransform);

                    unsigned int cInt = (color.r << 24) | (color.g << 16) | (color.b << 8) | color.a;
                    cubeInstances[cInt].push_back(mat);
                }
                break;
            }
            case ColliderType::Capsule: {
                float radius = c.capsule.radius;
                float halfHeight = c.capsule.halfHeight;
                const bool doOutline = outline && allowOutline;
                DrawCapsule(Vector3{0.0f, -halfHeight, 0.0f}, Vector3{0.0f, halfHeight, 0.0f}, radius, 6, 6, color);
                if (doOutline) {
                    DrawCapsuleWires(Vector3{0.0f, -halfHeight, 0.0f}, Vector3{0.0f, halfHeight, 0.0f}, radius, 12, 12, BLACK);
                }
                break;
            }
            case ColliderType::Convex: {
                if (c.polyhedron) {
                    drawConvex(*c.polyhedron, color, outline);
                }
                break;
            }
            case ColliderType::Mesh: {
                if (!c.mesh) break;
                const TriangleMesh& mesh = c.renderMesh ? *c.renderMesh : *c.mesh;
                drawTriangleMeshAtOrigin(mesh, allowOutline);
                break;
            }
            case ColliderType::Compound:
                break;
        }
    };

    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    applyQuat(body->orientation);

    Matrix matTrans = MatrixTranslate(pos.x, pos.y, pos.z);
    Matrix matRot = QuaternionToMatrix(Quaternion{body->orientation.x, body->orientation.y, body->orientation.z, body->orientation.w});
    Matrix bodyTransform = MatrixMultiply(matTrans, matRot);

    if (style.meshOverride) {
        drawTriangleMeshAtOrigin(*style.meshOverride, true);
        rlPopMatrix();

        if (isTransparent) {
            rlDrawRenderBatchActive();
            rlEnableDepthMask();
            EndBlendMode();
        }
        return;
    }
    
    switch (body->collider.type) {
        case ColliderType::Compound: {
            if (body->collider.compound) {
                const int childCount = (int)body->collider.compound->children.size();
                const bool suppressChildOutline = (childCount >= 12);

                for (const CompoundChild& child : body->collider.compound->children) {
                    rlPushMatrix();
                    
                    rlTranslatef(child.localPosition.x, child.localPosition.y, child.localPosition.z);
                    applyQuat(child.localOrientation);

                    Matrix childTrans = MatrixTranslate(child.localPosition.x, child.localPosition.y, child.localPosition.z);
                    Matrix childRot = QuaternionToMatrix(Quaternion{child.localOrientation.x, child.localOrientation.y, child.localOrientation.z, child.localOrientation.w});
                    Matrix childLocal = MatrixMultiply(childTrans, childRot);
                    Matrix childWorld = MatrixMultiply(childLocal, bodyTransform);

                    drawColliderAtOrigin(child.collider, !suppressChildOutline, childWorld);
                    
                    rlPopMatrix();
                }
            }
            break;
        }
        default:
            drawColliderAtOrigin(body->collider, true, bodyTransform);
            break;
    }

    rlPopMatrix();

    if (isTransparent) {
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        EndBlendMode();
    }
}

void Renderer::drawConvex(const PolyhedronShape& poly, Color color, bool outline) const {
    if (poly.verts.size() < 4) return;

    Color fillColor = color;

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

    if (outline) {
        rlBegin(RL_LINES);
        rlColor4ub(0, 0, 0, 255);
        for (const auto& e : poly.edges) {
            const Vec3& a = poly.verts[e.a];
            const Vec3& b = poly.verts[e.b];
            rlVertex3f(a.x, a.y, a.z);
            rlVertex3f(b.x, b.y, b.z);
        }
        rlEnd();
    }

    rlPopMatrix();
}

void Renderer::endFrame() {
    end3D();
    EndDrawing();

    Vector2 mouse = GetMousePosition();
    bool mouseOverUi = hasUiBlockRect && CheckCollisionPointRec(mouse, uiBlockRect);

    if (!mouseOverUi && IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
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

    if (!mouseOverUi) {
        float wheel = GetMouseWheelMove();
        if (fabsf(wheel) > 1e-6f) {
            camPos += forward * (wheel * 1.5f);
        }
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
