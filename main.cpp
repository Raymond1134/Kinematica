#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "physics/Spring.h"
#include "physics/collision/TriangleMesh.h"
#include "physics/Material.h"
#include "physics/SoftBody.h"
#include "physics/PhysicsFactory.h"
#include "physics/Raycast.h"
#include "render/Renderer.h"
#include "ui/UI.h"
#include "utils/JsonUtils.h"
#include "utils/MaterialLoader.h"
#include "utils/MeshGen.h"
#include <chrono>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <algorithm>
#include <utility>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <list>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <cstring>

struct DemoState {
    RigidBody* carBody = nullptr;
    int activeBlobIndex = -1;
};

static Color applyOpacity(Color c, float opacity) {
    float o = std::isfinite(opacity) ? opacity : 1.0f;
    o = std::clamp(o, 0.0f, 1.0f);
    c.a = (unsigned char)std::lround(o * 255.0f);
    return c;
}

static float getDeltaTime() {
    using clock = std::chrono::high_resolution_clock;
    static auto last = clock::now();
    auto now = clock::now();
    std::chrono::duration<float> dt = now - last;
    last = now;
    return dt.count();
}

enum class SpawnShape {
    Sphere,
    Cube,
    Capsule,
    Tetrahedron,
    Bipyramid,
    Dodecahedron,
    Ramp,
    Torus,
    Chain,
    Car,
    Cloth,
    Blob,
};

enum class SpawnMode {
    Throw,
    PlaceDynamic,
    PlaceStatic,
};

int main() {
    DemoState state;
    const float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;

    const int MAX_SUBSTEPS = 4;
    
    PhysicsWorld physicsWorld;
    std::list<RigidBody> dynamicBodies;
    PhysicsFactory physicsFactory(physicsWorld, dynamicBodies);
    std::vector<SoftBody> cloths;
    std::unordered_map<const RigidBody*, Renderer::RenderStyle> bodyStyles;
    static std::unordered_map<int, std::shared_ptr<TriangleMesh>> torusRenderCache;

    std::vector<MaterialProps> presetMaterials = MaterialLoader::loadMaterialsFromFolder("materials");
    MaterialProps customTemplate;
    customTemplate.name = "Custom";
    customTemplate.color = Color{155, 155, 155, 255};
    customTemplate.opacity = 1.0f;
    customTemplate.outline = true;

    MaterialProps currentMaterial = presetMaterials.empty() ? customTemplate : presetMaterials[0];
    bool currentMaterialIsCustom = presetMaterials.empty();

    MaterialProps floorMaterial = presetMaterials.empty() ? customTemplate : presetMaterials[0];
    bool floorMaterialIsCustom = presetMaterials.empty();

    for (const auto& m : presetMaterials) {
        if (m.name == "Plastic") {
            currentMaterial = m;
            floorMaterial = m;
            currentMaterialIsCustom = false;
            floorMaterialIsCustom = false;
            break;
        }
    }

    SpawnShape currentShape = SpawnShape::Cube;
    static int chainLength = 5;
    float currentSize = 0.35f;

    SpawnMode spawnMode = SpawnMode::Throw;


    Renderer renderer;
    if (!renderer.init(1280, 720, "Kinematica Sandbox")) return -1;
    int windowedW = 1280;
    int windowedH = 720;
    int windowedX = 0;
    int windowedY = 0;

    bool isPaused = false;
    bool singleStep = false;
    float timeScale = 1.0f;

    auto resetScene = [&]() {
        dynamicBodies.clear();
        physicsWorld.bodies.clear();
        cloths.clear();
        physicsWorld.springs.clear();
        physicsWorld.ballSocketJoints.clear();
        physicsWorld.hingeJoints.clear();
        bodyStyles.clear();
        state.carBody = nullptr;
        state.activeBlobIndex = -1;

        auto findMaterialByPrefix = [&](const char* prefix) -> const MaterialProps* {
            for (const auto& m : presetMaterials) {
                if (m.name.rfind(prefix, 0) == 0) return &m;
            }
            return nullptr;
        };

        const MaterialProps* woodMat = findMaterialByPrefix("Wood");
        const MaterialProps wallMaterial = woodMat ? *woodMat : customTemplate;

        constexpr int wallW = 10;
        constexpr int wallH = 10;
        constexpr int wallD = 2;
        const Vec3 half = {0.25f, 0.25f, 0.25f};
        const float spacingX = half.x * 2.0f;
        const float spacingY = half.y * 2.0f;
        const float spacingZ = half.z * 2.0f;

        const float baseX = -0.5f * (wallW - 1) * spacingX;
        const float baseY = physicsWorld.floorY + half.y;
        const float baseZ = -0.5f * (wallD - 1) * spacingZ;

        for (int z = 0; z < wallD; ++z) {
            for (int y = 0; y < wallH; ++y) {
                for (int x = 0; x < wallW; ++x) {
                    Vec3 pos = {baseX + x * spacingX, baseY + y * spacingY, baseZ + z * spacingZ};
                    
                    const float volume = (half.x * 2.0f) * (half.y * 2.0f) * (half.z * 2.0f);
                    float mass = std::max(0.05f, std::max(1.0f, wallMaterial.density) * std::max(0.0f, volume));
                    
                    RigidBody* rb = physicsFactory.CreateBox(pos, half, wallMaterial, mass, false, {0,0,0});
                    rb->sleeping = true;
                    rb->sleepTimer = 1.0f;

                    Renderer::RenderStyle st;
                    st.color = applyOpacity(wallMaterial.color, wallMaterial.opacity);
                    st.outline = wallMaterial.outline;
                    bodyStyles[rb] = st;
                }
            }
        }
    };

    resetScene();

    SetExitKey(0);

    while (!WindowShouldClose()) {
        auto removeRigidBody = [&](RigidBody* victim) {
            if (!victim) return;

            physicsWorld.springs.erase(
                std::remove_if(
                    physicsWorld.springs.begin(),
                    physicsWorld.springs.end(),
                    [&](const Spring& s) { return s.a == victim || s.b == victim; }),
                physicsWorld.springs.end());

            physicsWorld.ballSocketJoints.erase(
                std::remove_if(
                    physicsWorld.ballSocketJoints.begin(),
                    physicsWorld.ballSocketJoints.end(),
                    [&](const BallSocketJoint& j) { return j.a == victim || j.b == victim; }),
                physicsWorld.ballSocketJoints.end());

            physicsWorld.hingeJoints.erase(
                std::remove_if(
                    physicsWorld.hingeJoints.begin(),
                    physicsWorld.hingeJoints.end(),
                    [&](const HingeJoint& j) { return j.a == victim || j.b == victim; }),
                physicsWorld.hingeJoints.end());

            physicsWorld.bodies.erase(
                std::remove(physicsWorld.bodies.begin(), physicsWorld.bodies.end(), victim),
                physicsWorld.bodies.end());

            if (state.carBody == victim) state.carBody = nullptr;

            bodyStyles.erase(victim);

            for (auto it = dynamicBodies.begin(); it != dynamicBodies.end(); ++it) {
                if (&(*it) == victim) {
                    dynamicBodies.erase(it);
                    break;
                }
            }
        };

        if (IsKeyPressed(KEY_K)) {
            auto it = dynamicBodies.begin();
            while (it != dynamicBodies.end()) {
                if (!it->isStatic) {
                    it = dynamicBodies.erase(it);
                } else {
                    ++it;
                }
            }
            physicsWorld.bodies.clear();
            for (auto& b : dynamicBodies) {
                physicsWorld.addRigidBody(&b);
            }
            cloths.clear();
            physicsWorld.springs.clear();
            physicsWorld.ballSocketJoints.clear();
            physicsWorld.hingeJoints.clear();
            state.carBody = nullptr;
            state.activeBlobIndex = -1;
        }

        if (!UI::isBlocked() && (IsKeyPressed(KEY_X) || IsKeyPressed(KEY_DELETE))) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            if (camFwd.lengthSq() > 1e-12f) camFwd = camFwd.normalized();

            int softIndex = -1;
            float softT = 1e30f;
            {
                float t = 0.0f;
                Vec3 n = {0.0f, 1.0f, 0.0f};
                for (int i = 0; i < (int)cloths.size(); ++i) {
                    const SoftBody& sb = cloths[i];
                    if (sb.boundsRadius <= 0.0f) continue;
                    if (Raycast::raySphere(camPos, camFwd, sb.boundsCenter, sb.boundsRadius, t, n)) {
                        if (t > 1e-5f && t < softT) {
                            softT = t;
                            softIndex = i;
                        }
                    }
                }
            }

            Raycast::BodyPick pick = Raycast::pickBody(camPos, camFwd, dynamicBodies);
            const bool hitSoft = (softIndex >= 0 && softT < (pick.hit ? pick.t : 1e30f));

            if (hitSoft) {
                SoftBody& sb = cloths[softIndex];
                std::vector<RigidBody*> victims;
                victims.reserve(sb.particles.size());
                for (RigidBody* p : sb.particles) {
                    if (p) victims.push_back(p);
                }
                for (RigidBody* p : victims) {
                    removeRigidBody(p);
                }

                cloths.erase(cloths.begin() + softIndex);

                if (state.activeBlobIndex == softIndex) state.activeBlobIndex = -1;
                else if (state.activeBlobIndex > softIndex) state.activeBlobIndex -= 1;
                state.carBody = nullptr;
            } else if (pick.hit && pick.body) {
                removeRigidBody(pick.body);
                state.activeBlobIndex = -1;
            }
        }

        if (IsKeyPressed(KEY_R)) {
            resetScene();
        }

        if (IsKeyPressed(KEY_P)) {
            isPaused = !isPaused;
        }

        if (IsKeyPressed(KEY_PERIOD) && isPaused) {
            singleStep = true;
        }

        if (IsKeyPressed(KEY_LEFT_BRACKET)) {
            timeScale = std::max(0.1f, timeScale - 0.1f);
        }
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
            timeScale = std::min(5.0f, timeScale + 0.1f);
        }

        if ((IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) && IsKeyPressed(KEY_ENTER)) {
            if (!IsWindowFullscreen()) {
                windowedW = GetScreenWidth();
                windowedH = GetScreenHeight();
                windowedX = GetWindowPosition().x;
                windowedY = GetWindowPosition().y;

                int mon = GetCurrentMonitor();
                int mw = GetMonitorWidth(mon);
                int mh = GetMonitorHeight(mon);

                SetWindowSize(mw, mh);
                SetWindowPosition(0, 0);
                ToggleFullscreen();
            } else {
                ToggleFullscreen();
                SetWindowSize(windowedW, windowedH);
                SetWindowPosition(windowedX, windowedY);
            }
        }
        float frameTime = getDeltaTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        
        if (!isPaused) {
            accumulator += frameTime * timeScale;
        } else if (singleStep) {
            accumulator += FIXED_DT;
            singleStep = false;
        }

        physicsWorld.floorFriction = std::clamp(floorMaterial.friction, 0.0f, 2.0f);
        physicsWorld.floorRestitution = std::clamp(floorMaterial.restitution, 0.0f, 1.0f);

        Vec3 frameCamPos = renderer.getCameraPosition();
        Vec3 frameCamFwd = renderer.getCameraForward();
        if (frameCamFwd.lengthSq() > 1e-12f) frameCamFwd = frameCamFwd.normalized();
        RayHit frameRayHit = Raycast::raycastWorldPlacement(frameCamPos, frameCamFwd, physicsWorld.floorY, dynamicBodies);
        Vec3 framePlacePoint = frameCamPos + frameCamFwd * 2.0f;
        Vec3 framePlaceNormal = {0.0f, 1.0f, 0.0f};
        if (frameRayHit.hit) { framePlacePoint = frameRayHit.point; framePlaceNormal = frameRayHit.normal; }

        Vec3 lastHologramPos = framePlacePoint;
        Quat lastHologramOri = Quat::identity();
        bool hologramValid = false;
        {
            RigidBody tmp;
            tmp.position = framePlacePoint;
            tmp.orientation = Quat::identity();
            float s = std::clamp(currentSize, 0.10f, 1.25f);
            switch (currentShape) {
                case SpawnShape::Sphere:
                    tmp.collider = Collider::createSphere(s);
                    break;
                case SpawnShape::Cube:
                    tmp.collider = Collider::createBox({s, s, s});
                    break;
                case SpawnShape::Capsule: {
                    float hh = s * 0.6f;
                    float r = s * 0.33f;
                    tmp.collider = Collider::createCapsule(r, hh);
                    break;
                }
                case SpawnShape::Tetrahedron: {
                    auto verts = MeshGen::makeTetraVerts(s);
                    tmp.collider = Collider::createConvex(verts);
                    break;
                }
                default: {
                    tmp.collider = Collider::createBox({s, s, s});
                    break;
                }
            }

            float off = Raycast::placementOffsetAlongNormal(tmp.collider, tmp.orientation, framePlaceNormal);
            lastHologramPos = framePlacePoint + framePlaceNormal.normalized() * (off + 0.002f);
            lastHologramOri = tmp.orientation;
            hologramValid = true;
        }

        if (IsKeyPressed(KEY_Q)) {
            const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            Vec3 camPos = frameCamPos;
            Vec3 camFwd = frameCamFwd;

            Vec3 up = {0.0f, 1.0f, 0.0f};
            Vec3 right = Vec3::cross(up, camFwd).normalized();
            if (right.lengthSq() < 1e-8f) right = {1.0f, 0.0f, 0.0f};

            if (currentShape == SpawnShape::Chain) {
                Vec3 startPos;
                if (spawnMode == SpawnMode::Throw) {
                    startPos = camPos + camFwd * 2.0f;
                } else {
                    if (hologramValid) startPos = lastHologramPos;
                    else startPos = framePlacePoint + framePlaceNormal * 0.1f;
                }

                float radius = currentSize * 0.5f;
                float spacing = radius * 2.05f;
                if (spawnMode == SpawnMode::PlaceStatic) {
                    startPos.y += (float)(chainLength - 1) * spacing;
                }
                
                Vec3 vel = {0,0,0};
                if (spawnMode == SpawnMode::Throw) {
                    vel = camFwd * 10.0f;
                }

                bool startStatic = (spawnMode == SpawnMode::PlaceStatic);
                auto chain = physicsFactory.CreateChain(startPos, {0.0f, -1.0f, 0.0f}, chainLength, radius, spacing, currentMaterial, startStatic, vel);
                
                Renderer::RenderStyle st;
                st.color = applyOpacity(currentMaterial.color, currentMaterial.opacity);
                st.outline = currentMaterial.outline;
                
                for (auto* b : chain) {
                    bodyStyles[b] = st;
                }
            } else if (currentShape == SpawnShape::Car) {
                Vec3 startPos;
                if (spawnMode == SpawnMode::Throw) {
                    startPos = camPos + camFwd * 2.0f;
                } else {
                    if (hologramValid) startPos = lastHologramPos;
                    else startPos = framePlacePoint + framePlaceNormal * 0.5f;
                }
                
                bool isStatic = (spawnMode == SpawnMode::PlaceStatic);
                Vec3 vel = {0,0,0};
                if (spawnMode == SpawnMode::Throw) {
                    vel = camFwd * 10.0f;
                }

                RigidBody* chassisPtr = physicsFactory.CreateCar(startPos, 1.0f, currentMaterial, isStatic, vel);

                if (spawnMode != SpawnMode::Throw) {
                    Vec3 f = {camFwd.x, 0.0f, camFwd.z};
                    if (f.lengthSq() > 1e-8f) {
                        f = f.normalized();
                        float yaw = atan2f(f.x, f.z);
                        chassisPtr->orientation = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, yaw);
                    }
                }
                
                state.carBody = chassisPtr;
                printf("Car spawned at (%.2f, %.2f, %.2f)\n", chassisPtr->position.x, chassisPtr->position.y, chassisPtr->position.z);
                
                Renderer::RenderStyle st;
                st.color = currentMaterial.color;
                st.color.a = (unsigned char)(currentMaterial.opacity * 255.0f);
                st.outline = currentMaterial.outline;
                bodyStyles[chassisPtr] = st;
            } else if (currentShape == SpawnShape::Cloth) {
                Vec3 startPos;
                if (spawnMode == SpawnMode::Throw) {
                    startPos = camPos + camFwd * 3.0f;
                } else {
                    if (hologramValid) startPos = lastHologramPos;
                    else startPos = framePlacePoint + framePlaceNormal * 1.0f;
                }

                int rows = 14;
                int cols = 14;
                float spacing = 0.2f * std::clamp(currentSize, 0.5f, 2.0f);
                
                Vec3 vel = {0,0,0};
                if (spawnMode == SpawnMode::Throw) {
                    vel = camFwd * 10.0f;
                }
                
                bool isStatic = (spawnMode == SpawnMode::PlaceStatic);

                SoftBody cloth = physicsFactory.CreateCloth(startPos, rows, cols, spacing, currentMaterial, isStatic, vel);
                cloths.push_back(cloth);
            } else if (currentShape == SpawnShape::Blob) {
                Vec3 startPos;
                if (spawnMode == SpawnMode::Throw) {
                    startPos = camPos + camFwd * 3.0f;
                } else {
                    if (hologramValid) startPos = lastHologramPos;
                    else startPos = framePlacePoint + framePlaceNormal * 1.5f;
                }

                int dim = 3;
                float spacing = 0.4f * std::clamp(currentSize, 0.5f, 2.0f);
                float radius = spacing * 0.6f;
                
                Vec3 vel = {0,0,0};
                if (spawnMode == SpawnMode::Throw) {
                    vel = camFwd * 10.0f;
                }
                
                bool isStatic = (spawnMode == SpawnMode::PlaceStatic);

                SoftBody blob = physicsFactory.CreateBlob(startPos, dim, spacing, radius, currentMaterial, isStatic, vel);
                cloths.push_back(blob);
                state.activeBlobIndex = (int)cloths.size() - 1;
                state.carBody = nullptr;
            } else {
                RigidBody b;
                b.position = framePlacePoint;
                b.velocity = {0.0f, 0.0f, 0.0f};
                b.orientation = Quat::identity();
                b.angularVelocity = {0.0f, 0.0f, 0.0f};

                if (spawnMode == SpawnMode::Throw) {
                    b.position = camPos + camFwd * 2.0f;
                    b.velocity = camFwd * 10.0f;

                    float tiltDeg = (float)GetRandomValue(-5, 5);
                    float rollDeg = (float)GetRandomValue(-5, 5);
                    Quat qTilt = Quat::fromAxisAngle(right, tiltDeg * DEG2RAD);
                    Quat qRoll = Quat::fromAxisAngle(camFwd, rollDeg * DEG2RAD);
                    b.orientation = (qRoll * qTilt).normalized();

                    float spin = 7.0f;
                    float spinJitter = (float)GetRandomValue(-5, 5) / 5.0f;
                    b.angularVelocity = right * (spin * (1.0f + 0.25f * spinJitter)) + up * (1.0f * spinJitter);
                }

                if (spawnMode != SpawnMode::Throw) {
                    if (hologramValid) {
                        b.position = lastHologramPos;
                        b.orientation = lastHologramOri;
                    } else {
                        b.position = framePlacePoint;
                    }
                }

                float s = std::clamp(currentSize, 0.10f, 1.25f);
                
                Vec3 vel = {0,0,0};
                Vec3 angVel = {0,0,0};
                Quat orientation = Quat::identity();
                
                if (spawnMode == SpawnMode::Throw) {
                    vel = camFwd * 10.0f;
                    float tiltDeg = (float)GetRandomValue(-5, 5);
                    float rollDeg = (float)GetRandomValue(-5, 5);
                    Quat qTilt = Quat::fromAxisAngle(right, tiltDeg * DEG2RAD);
                    Quat qRoll = Quat::fromAxisAngle(camFwd, rollDeg * DEG2RAD);
                    orientation = (qRoll * qTilt).normalized();

                    float spin = 7.0f;
                    float spinJitter = (float)GetRandomValue(-5, 5) / 5.0f;
                    angVel = right * (spin * (1.0f + 0.25f * spinJitter)) + up * (1.0f * spinJitter);
                }

                bool isStatic = (spawnMode == SpawnMode::PlaceStatic);
                
                float density = std::max(1.0f, currentMaterial.density);
                float mass = 1.0f;

                auto calcVolume = [&](float vol) {
                    return std::max(0.05f, density * vol);
                };

                RigidBody* rb = nullptr;

                switch (currentShape) {
                    case SpawnShape::Sphere: {
                        float vol = (4.0f / 3.0f) * 3.14159265f * s * s * s;
                        rb = physicsFactory.CreateSphere(b.position, s, currentMaterial, calcVolume(vol), isStatic, vel);
                        break;
                    }
                    case SpawnShape::Cube: {
                        float vol = (2.0f * s) * (2.0f * s) * (2.0f * s);
                        rb = physicsFactory.CreateBox(b.position, {s, s, s}, currentMaterial, calcVolume(vol), isStatic, vel);
                        break;
                    }
                    case SpawnShape::Capsule: {
                        float r = s;
                        float hh = 1.5f * s;
                        float cylH = 2.0f * hh;
                        float vol = 3.14159265f * r * r * cylH + (4.0f / 3.0f) * 3.14159265f * r * r * r;
                        rb = physicsFactory.CreateCapsule(b.position, r, hh, currentMaterial, calcVolume(vol), isStatic, vel);
                        break;
                    }
                    case SpawnShape::Tetrahedron: {
                        auto verts = MeshGen::makeTetraVerts(s);

                        rb = physicsFactory.CreateConvex(b.position, verts, currentMaterial, 1.0f, isStatic, vel);
                        if (rb->collider.polyhedron) {
                            float vol = rb->collider.polyhedron->computeVolume();
                            rb->mass = calcVolume(vol);
                            if (isStatic) rb->mass = 0.0f;
                        }
                        break;
                    }
                    case SpawnShape::Bipyramid: {
                        auto verts = MeshGen::makePentagonalBipyramidVerts(s);
                        rb = physicsFactory.CreateConvex(b.position, verts, currentMaterial, 1.0f, isStatic, vel);
                        if (rb->collider.polyhedron) {
                            float vol = rb->collider.polyhedron->computeVolume();
                            rb->mass = calcVolume(vol);
                            if (isStatic) rb->mass = 0.0f;
                        }
                        break;
                    }
                    case SpawnShape::Dodecahedron: {
                        auto verts = MeshGen::makeDodecaVerts(s);
                        rb = physicsFactory.CreateConvex(b.position, verts, currentMaterial, 1.0f, isStatic, vel);
                        if (rb->collider.polyhedron) {
                            float vol = rb->collider.polyhedron->computeVolume();
                            rb->mass = calcVolume(vol);
                            if (isStatic) rb->mass = 0.0f;
                        }
                        break;
                    }
                    case SpawnShape::Ramp: {
                        float width = 2.5f * s;
                        float length = 4.0f * s;
                        float height = 1.4f * s;
                        auto rampRender = MeshGen::makeRampMesh(width, length, height);
                        auto rampCollision = shiftDown ? rampRender : MeshGen::makeRampCollisionMesh(width, length, height);

                        Vec3 f = {camFwd.x, 0.0f, camFwd.z};
                        if (f.lengthSq() < 1e-8f) f = {0.0f, 0.0f, 1.0f};
                        f = f.normalized();
                        float yaw = atan2f(f.x, f.z);
                        orientation = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, yaw).normalized();
                        
                        float vol = width * length * std::max(0.01f, height) * 0.5f;
                        rb = physicsFactory.CreateMesh(b.position, rampCollision, currentMaterial, calcVolume(vol), isStatic, vel);
                        rb->collider.renderMesh = rampRender;
                        break;
                    }
                    case SpawnShape::Torus: {
                        float majorR = s * 1.0f;
                        float minorR = s * 0.45f;
                        std::vector<CompoundChild> children;
                        const int segments = 64;
                        children.reserve(segments);

                        auto quatFromTo = [](const Vec3& from, const Vec3& to) -> Quat {
                            Vec3 f = from.normalized();
                            Vec3 t = to.normalized();
                            float c = Vec3::dot(f, t);
                            if (c >= 1.0f - 1e-6f) return Quat::identity();
                            if (c <= -1.0f + 1e-6f) {
                                Vec3 ortho = (fabsf(f.x) < 0.9f) ? Vec3{1.0f, 0.0f, 0.0f} : Vec3{0.0f, 0.0f, 1.0f};
                                Vec3 axis = Vec3::cross(f, ortho).normalized();
                                return Quat::fromAxisAngle(axis, 3.14159265f).normalized();
                            }
                            Vec3 axis = Vec3::cross(f, t);
                            Quat q{1.0f + c, axis.x, axis.y, axis.z};
                            return q.normalized();
                        };

                        for (int i = 0; i < segments; ++i) {
                            float theta1 = (float)i / (float)segments * 2.0f * 3.14159265f;
                            float theta2 = (float)(i + 1) / (float)segments * 2.0f * 3.14159265f;

                            Vec3 c1 = {majorR * cosf(theta1), 0.0f, majorR * sinf(theta1)};
                            Vec3 c2 = {majorR * cosf(theta2), 0.0f, majorR * sinf(theta2)};
                            Vec3 mid = (c1 + c2) * 0.5f;
                            Vec3 d = (c2 - c1);
                            float len = d.length();
                            if (len < 1e-6f) continue;
                            Vec3 dir = d * (1.0f / len);

                            CompoundChild child;
                            child.collider = Collider::createCapsule(minorR, 0.5f * len);
                            child.localPosition = mid;
                            child.localOrientation = quatFromTo({0.0f, 1.0f, 0.0f}, dir);
                            children.push_back(child);
                        }

                        float vol = 2.0f * 3.14159f * 3.14159f * majorR * minorR * minorR;
                        rb = physicsFactory.CreateCompound(b.position, children, currentMaterial, calcVolume(vol), isStatic, vel);

                        // Explicit inertia tensor for Torus
                        float M = rb->mass;
                        float R = majorR;
                        float r = minorR;
                        float I_axis = M * (R*R + 0.75f*r*r);
                        float I_perp = M * (0.5f*R*R + 0.625f*r*r);
                        float inertiaScale = 1.2f;
                        I_axis *= inertiaScale;
                        I_perp *= inertiaScale;

                        rb->useExplicitInertia = true;
                        rb->explicitInvInertia = {
                            (I_perp > 0.0001f) ? 1.0f/I_perp : 0.0f,
                            (I_axis > 0.0001f) ? 1.0f/I_axis : 0.0f,
                            (I_perp > 0.0001f) ? 1.0f/I_perp : 0.0f
                        };
                        break;
                    }
                }

                if (rb) {
                    rb->orientation = orientation;
                    rb->angularVelocity = angVel;
                    rb->sleeping = (spawnMode != SpawnMode::Throw);
                    rb->sleepTimer = (spawnMode != SpawnMode::Throw) ? 1.0f : 0.0f;

                    if (spawnMode != SpawnMode::Throw) {
                        if (hologramValid) {
                            rb->position = lastHologramPos;
                            rb->orientation = lastHologramOri;
                        } else {
                            float offset = Raycast::placementOffsetAlongNormal(rb->collider, rb->orientation, framePlaceNormal);
                            rb->position = framePlacePoint + framePlaceNormal.normalized() * (offset + 0.002f);
                        }
                        rb->velocity = {0.0f, 0.0f, 0.0f};
                        rb->angularVelocity = {0.0f, 0.0f, 0.0f};
                    }

                    Renderer::RenderStyle st;
                    st.color = applyOpacity(currentMaterial.color, currentMaterial.opacity);
                    st.outline = currentMaterial.outline;

                    if (currentShape == SpawnShape::Torus) {
                        float s = std::clamp(currentSize, 0.10f, 1.25f);
                        int key = (int)lrintf(s * 100.0f);
                        auto it = torusRenderCache.find(key);
                        if (it == torusRenderCache.end()) {
                            float majorR = s * 1.0f;
                            float minorR = s * 0.45f;
                            auto mesh = MeshGen::makeTorusMesh(majorR, minorR, 64, 16);
                            it = torusRenderCache.emplace(key, std::move(mesh)).first;
                        }
                        st.meshOverride = it->second;
                    }

                    bodyStyles[rb] = st;
                }

                if (currentShape == SpawnShape::Car) {
                    state.carBody = &dynamicBodies.back();
                    state.activeBlobIndex = -1;
                } else {
                    state.carBody = nullptr;
                    state.activeBlobIndex = -1;
                }
            }
        }
        {
            Vec3 camFwd = renderer.getCameraForward();
            camFwd.y = 0.0f;
            if (camFwd.lengthSq() > 1e-6f) camFwd = camFwd.normalized();
            else camFwd = {0.0f, 0.0f, 1.0f};
            
            Vec3 camRight = Vec3::cross(camFwd, {0.0f, 1.0f, 0.0f}).normalized();
            
            Vec3 moveDir = {0.0f, 0.0f, 0.0f};
            if (IsKeyDown(KEY_UP)) moveDir += camFwd;
            if (IsKeyDown(KEY_DOWN)) moveDir -= camFwd;
            if (IsKeyDown(KEY_LEFT)) moveDir -= camRight;
            if (IsKeyDown(KEY_RIGHT)) moveDir += camRight;

            if (moveDir.lengthSq() > 1e-6f) {
                moveDir = moveDir.normalized();
                float accelMag = 12.0f;
                
                if (state.activeBlobIndex >= 0 && state.activeBlobIndex < (int)cloths.size()) {
                    auto& sb = cloths[state.activeBlobIndex];
                    if (sb.isBlob) {
                        for (RigidBody* p : sb.particles) {
                            if (p && !p->isStatic) {
                                p->velocity += moveDir * (accelMag * FIXED_DT);
                                p->sleeping = false;
                            }
                        }
                    }
                }
            }
        }

        // Car Controls
        if (state.carBody && !state.carBody->isStatic) {
            const float accelForce = 60.0f;
            const float turnSpeed = 4.5f;
            const float lateralFriction = 8.0f;
            const float maxSpeed = 40.0f;

            Vec3 fwd = state.carBody->orientation.rotate({0.0f, 0.0f, 1.0f});
            Vec3 right = state.carBody->orientation.rotate({1.0f, 0.0f, 0.0f});

            float currentSpeed = Vec3::dot(state.carBody->velocity, fwd);
            if (IsKeyDown(KEY_UP)) {
                if (currentSpeed < maxSpeed) {
                    state.carBody->velocity += fwd * (accelForce * FIXED_DT);
                }
                state.carBody->sleeping = false;
            }
            if (IsKeyDown(KEY_DOWN)) {
                if (currentSpeed > -maxSpeed) {
                    state.carBody->velocity -= fwd * (accelForce * FIXED_DT);
                }
                state.carBody->sleeping = false;
            }

            float speed = state.carBody->velocity.length();
            if (speed > 0.5f) {
                float dir = Vec3::dot(state.carBody->velocity, fwd) > 0.0f ? 1.0f : -1.0f;
                float turnFactor = std::min(speed / 10.0f, 1.0f); 
                
                if (IsKeyDown(KEY_LEFT)) {
                    state.carBody->angularVelocity.y += turnSpeed * dir * turnFactor * FIXED_DT;
                    state.carBody->sleeping = false;
                }
                if (IsKeyDown(KEY_RIGHT)) {
                    state.carBody->angularVelocity.y -= turnSpeed * dir * turnFactor * FIXED_DT;
                    state.carBody->sleeping = false;
                }
            }
            Vec3 latVel = right * Vec3::dot(state.carBody->velocity, right);
            state.carBody->velocity -= latVel * (lateralFriction * FIXED_DT);

            if (state.carBody->collider.compound) {
                float wheelRadius = 0.3f;
                float dist = Vec3::dot(state.carBody->velocity, fwd) * FIXED_DT;
                float angleDelta = dist / wheelRadius;
                
                for (size_t i = 1; i < state.carBody->collider.compound->children.size(); ++i) {
                    if (i > 4) break;
                    CompoundChild& wheel = state.carBody->collider.compound->children[i];
                    Quat rot = Quat::fromAxisAngle({1.0f, 0.0f, 0.0f}, angleDelta);
                    wheel.localOrientation = (wheel.localOrientation * rot).normalized();
                }
            }
        }

        float hingeMotorSpeed = 0.0f;
        if (IsKeyDown(KEY_UP)) hingeMotorSpeed = 15.0f;
        if (IsKeyDown(KEY_DOWN)) hingeMotorSpeed = -15.0f;
        
        for (auto& j : physicsWorld.hingeJoints) {
            if (j.enableMotor) {
                j.motorTargetVelocity = hingeMotorSpeed;
                if (hingeMotorSpeed != 0.0f) {
                    if (j.a) j.a->sleeping = false;
                    if (j.b) j.b->sleeping = false;
                }
            }
        }

        physicsWorld.floorFriction = floorMaterial.friction;
        physicsWorld.floorRestitution = floorMaterial.restitution;

        int substeps = 0;
        while (accumulator >= FIXED_DT && substeps < MAX_SUBSTEPS) {
            physicsWorld.step(FIXED_DT);
            accumulator -= FIXED_DT;
            ++substeps;
        }
        if (substeps == MAX_SUBSTEPS && accumulator >= FIXED_DT) {
            accumulator = 0.0f;
        }

        renderer.beginFrame();
        {
            const Vec3 camPos = renderer.getCameraPosition();

            auto getStyle = [&](const RigidBody* rb) -> Renderer::RenderStyle {
                auto it = bodyStyles.find(rb);
                if (it != bodyStyles.end()) return it->second;
                Renderer::RenderStyle st;
                st.color = rb->isStatic ? Color{140, 140, 140, 255} : Color{60, 120, 200, 255};
                st.outline = false;
                return st;
            };

            std::vector<const RigidBody*> opaque;
            std::vector<const RigidBody*> transparent;
            opaque.reserve(dynamicBodies.size());
            transparent.reserve(dynamicBodies.size());

            for (auto& body : dynamicBodies) {
                if (!body.visible) continue;
                Renderer::RenderStyle st = getStyle(&body);
                if (st.color.a < 255) transparent.push_back(&body);
                else opaque.push_back(&body);
            }

            for (const RigidBody* rb : opaque) {
                renderer.drawRigidBody(rb, getStyle(rb));
            }

            renderer.flush();

            std::sort(transparent.begin(), transparent.end(), [&](const RigidBody* a, const RigidBody* b) {
                Vec3 da = a->position - camPos;
                Vec3 db = b->position - camPos;
                return da.lengthSq() > db.lengthSq();
            });
            for (const RigidBody* rb : transparent) {
                renderer.drawRigidBody(rb, getStyle(rb));
            }

            for (auto& cloth : cloths) {
                cloth.update();
                cloth.draw(renderer);
            }

            if (spawnMode != SpawnMode::Throw) {
                Vec3 camFwd = renderer.getCameraForward().normalized();
                const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
                Vec3 placePoint = framePlacePoint;
                Vec3 placeNormal = framePlaceNormal;

                Renderer::RenderStyle holo;
                holo.color = Color{0, 190, 255, 90};
                holo.outline = true;

                if (currentShape == SpawnShape::Chain) {
                    Vec3 startPos = placePoint + placeNormal * 0.1f;
                    
                    float radius = currentSize * 0.5f;
                    float spacing = radius * 2.05f;

                    if (spawnMode == SpawnMode::PlaceStatic) {
                        startPos.y += (float)(chainLength - 1) * spacing;
                    }
                    lastHologramPos = startPos;
                    lastHologramOri = Quat::identity();
                    hologramValid = true;
                    
                    for (int i = 0; i < chainLength; ++i) {
                        RigidBody linkGhost;
                        linkGhost.position = startPos + Vec3{0.0f, -i * spacing, 0.0f};
                        linkGhost.collider = Collider::createSphere(radius);
                        linkGhost.orientation = Quat::identity();
                        linkGhost.velocity = {0.0f, 0.0f, 0.0f};
                        linkGhost.angularVelocity = {0.0f, 0.0f, 0.0f};
                        renderer.drawRigidBody(&linkGhost, holo);
                    }
                } else {
                    RigidBody ghost;
                    ghost.position = placePoint;
                    ghost.velocity = {0.0f, 0.0f, 0.0f};
                    ghost.angularVelocity = {0.0f, 0.0f, 0.0f};
                    ghost.orientation = Quat::identity();

                    float s = std::clamp(currentSize, 0.10f, 1.25f);
                    switch (currentShape) {
                        case SpawnShape::Sphere:
                            ghost.collider = Collider::createSphere(s);
                            break;
                        case SpawnShape::Cube:
                            ghost.collider = Collider::createBox({s, s, s});
                            break;
                        case SpawnShape::Capsule:
                            ghost.collider = Collider::createCapsule(s, 1.5f * s);
                            break;
                        case SpawnShape::Tetrahedron:
                            ghost.collider = Collider::createConvex(MeshGen::makeTetraVerts(s));
                            break;
                        case SpawnShape::Bipyramid:
                            ghost.collider = Collider::createConvex(MeshGen::makePentagonalBipyramidVerts(s));
                            break;
                        case SpawnShape::Dodecahedron:
                            ghost.collider = Collider::createConvex(MeshGen::makeDodecaVerts(s));
                            break;
                        case SpawnShape::Torus: {
                            float majorR = s * 1.0f;
                            float minorR = s * 0.45f;
                            auto torusMesh = MeshGen::makeTorusMesh(majorR, minorR, 64, 16);
                            ghost.collider = Collider::createMesh(torusMesh, torusMesh);
                            break;
                        }
                        case SpawnShape::Ramp: {
                            float width = 2.5f * s;
                            float length = 4.0f * s;
                            float height = 1.4f * s;
                            auto rampRender = MeshGen::makeRampMesh(width, length, height);
                            auto rampCollision = shiftDown ? rampRender : MeshGen::makeRampCollisionMesh(width, length, height);
                            ghost.collider = Collider::createMesh(rampCollision, rampRender);
                            Vec3 f = {camFwd.x, 0.0f, camFwd.z};
                            if (f.lengthSq() < 1e-8f) f = {0.0f, 0.0f, 1.0f};
                            f = f.normalized();
                            float yaw = atan2f(f.x, f.z);
                            ghost.orientation = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, yaw).normalized();
                            break;
                        }
                        case SpawnShape::Car: {
                            Vec3 chassisDim = {1.0f, 0.25f, 2.0f};
                            float wheelRadius = 0.3f;
                            Vec3 wheelOffsets[4] = {
                                {-0.6f, -0.1f, -0.7f},
                                { 0.6f, -0.1f, -0.7f},
                                {-0.6f, -0.1f,  0.7f},
                                { 0.6f, -0.1f,  0.7f}
                            };

                            std::vector<CompoundChild> carChildren;
                            carChildren.reserve(5);
                            {
                                CompoundChild body;
                                body.collider = Collider::createBox(chassisDim * 0.5f);
                                body.localPosition = {0.0f, 0.0f, 0.0f};
                                body.localOrientation = Quat::identity();
                                carChildren.push_back(body);
                            }
                            for (int i = 0; i < 4; ++i) {
                                CompoundChild w;
                                w.collider = Collider::createSphere(wheelRadius);
                                w.localPosition = wheelOffsets[i];
                                w.localOrientation = Quat::identity();
                                carChildren.push_back(w);
                            }
                            ghost.collider = Collider::createCompound(carChildren);
                            
                            Vec3 f = {camFwd.x, 0.0f, camFwd.z};
                            if (f.lengthSq() < 1e-8f) f = {0.0f, 0.0f, 1.0f};
                            f = f.normalized();
                            float yaw = atan2f(f.x, f.z);
                            ghost.orientation = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, yaw).normalized();
                            break;
                        }
                        case SpawnShape::Cloth: {
                            float spacing = 0.3f * std::clamp(currentSize, 0.5f, 2.0f);
                            float dim = 9.0f * spacing;
                            ghost.collider = Collider::createBox({dim * 0.5f, 0.1f, dim * 0.5f});
                            break;
                        }
                        case SpawnShape::Blob: {
                            float spacing = 0.4f * std::clamp(currentSize, 0.5f, 2.0f);
                            float dim = spacing;
                            ghost.collider = Collider::createBox({dim, dim, dim});
                            break;
                        }
                    }

                    float offset = Raycast::placementOffsetAlongNormal(ghost.collider, ghost.orientation, placeNormal);
                    ghost.position = placePoint + placeNormal.normalized() * (offset + 0.002f);

                    lastHologramPos = ghost.position;
                    lastHologramOri = ghost.orientation;
                    hologramValid = true;

                    renderer.drawRigidBody(&ghost, holo);
                }
            }
        }
        renderer.end3D();
        {
            static bool hudCollapsed = false;
            int sw = GetScreenWidth();
            int sh = GetScreenHeight();

            float panelW = hudCollapsed ? 34.0f : std::min(360.0f, std::max(240.0f, sw * 0.28f));
            panelW = std::min(panelW, (float)sw);

            Rectangle panel = {(float)sw - panelW, 0.0f, panelW, (float)sh};
            renderer.setUiBlockRect(panel);

            DrawRectangleRec(panel, Color{18, 18, 18, 235});
            DrawRectangleLinesEx(panel, 1.0f, Color{60, 60, 60, 255});

            Rectangle toggle = {panel.x + 7.0f, panel.y + 8.0f, panel.width - 14.0f, 26.0f};
            if (UI::Button(toggle, hudCollapsed ? "<" : ">")) {
                hudCollapsed = !hudCollapsed;
            }

            if (hudCollapsed) {
                renderer.setUiBlockRect(panel);
                renderer.endFrame();
                continue;
            }

            const float pad = 12.0f;
            const float footerH = 42.0f;
            const float headerH = 40.0f;

            static float scrollY = 0.0f;
            static float contentHeight = 1000.0f;
            static int matSelIndex = 0;
            static int floorSelIndex = 0;
            float viewHeight = panel.height - footerH - headerH;

            if (CheckCollisionPointRec(GetMousePosition(), panel)) {
                scrollY -= GetMouseWheelMove() * 30.0f;
            }
            float maxScroll = std::max(0.0f, contentHeight - viewHeight);
            scrollY = std::clamp(scrollY, 0.0f, maxScroll);

            Rectangle scissorRect = {panel.x, panel.y + headerH, panel.width, viewHeight};
            UI::setClipRect(scissorRect);
            BeginScissorMode((int)scissorRect.x, (int)scissorRect.y, (int)scissorRect.width, (int)scissorRect.height);
            
            float x = panel.x + pad;
            float startY = panel.y + headerH + pad;
            float y = startY - scrollY;

            DrawText("Actions", (int)x, (int)y, 22, RAYWHITE);
            y += 24.0f;
            {
                float btnW = (panel.width - 24.0f) * 0.5f - 4.0f;
                if (UI::Button({x, y, btnW, 24.0f}, "Kill All")) {
                    physicsWorld.bodies.clear();
                    dynamicBodies.clear();
                    cloths.clear();
                    physicsWorld.springs.clear();
                    physicsWorld.ballSocketJoints.clear();
                    physicsWorld.hingeJoints.clear();
                    state.carBody = nullptr;
                    state.activeBlobIndex = -1;
                }
                if (UI::Button({x + btnW + 8.0f, y, btnW, 24.0f}, "Reset")) {
                    resetScene();
                }
                y += 30.0f;

                if (UI::Button({x, y, btnW, 24.0f}, isPaused ? "Resume" : "Pause")) {
                    isPaused = !isPaused;
                }
                if (isPaused) {
                    if (UI::Button({x + btnW + 8.0f, y, btnW, 24.0f}, "Step")) {
                        singleStep = true;
                    }
                }
                y += 30.0f;

                DrawText("Time Scale", (int)x, (int)y + 4, 16, Color{200, 200, 200, 255});
                if (UI::Button({x + 90.0f, y, 24.0f, 24.0f}, "-")) {
                    timeScale = std::max(0.1f, timeScale - 0.1f);
                }
                char tsBuf[32];
                snprintf(tsBuf, sizeof(tsBuf), "%.1f", timeScale);
                DrawText(tsBuf, (int)(x + 122.0f), (int)y + 4, 16, RAYWHITE);
                if (UI::Button({x + 154.0f, y, 24.0f, 24.0f}, "+")) {
                    timeScale = std::min(5.0f, timeScale + 0.1f);
                }
                y += 34.0f;
            }

            DrawText("Spawner", (int)x, (int)y, 22, RAYWHITE);
            y += 24.0f;
            {
                std::vector<std::string> modes = {"Throw (Dynamic)", "Place (Dynamic)", "Place (Static)"};
                int sel = (int)spawnMode;
                Rectangle r = {x, y, panel.width - 24.0f, 28.0f};
                UI::Dropdown(100, r, modes, (int*)&spawnMode);
                y += 34.0f;
            }

            DrawText("Shape", (int)x, (int)y, 18, RAYWHITE);
            y += 24.0f;
            {
                std::vector<std::string> shapes = {
                    "Sphere", "Cube", "Capsule", "Tetrahedron", 
                    "Bipyramid", "Dodecahedron", "Ramp", "Torus", "Chain", "Car", "Cloth", "Blob"
                };
                int sel = (int)currentShape;
                Rectangle r = {x, y, panel.width - 24.0f, 28.0f};
                UI::Dropdown(101, r, shapes, (int*)&currentShape);
                y += 34.0f;
            }

            if (currentShape == SpawnShape::Chain) {
                DrawText("Chain Length", (int)x, (int)y, 16, Color{200, 200, 200, 255});
                y += 18.0f;
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 14.0f};
                float val = (float)chainLength;
                val = UI::Slider(8888, sld, val, 2.0f, 20.0f, changed);
                chainLength = (int)val;
                y += 18.0f;
                char buf[32];
                snprintf(buf, sizeof(buf), "%d", chainLength);
                DrawText(buf, (int)x, (int)y, 14, Color{170, 170, 170, 255});
                y += 22.0f;
            }

            y += 8.0f;
            DrawText("Size", (int)x, (int)y, 18, RAYWHITE);
            y += 22.0f;
            {
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 16.0f};
                currentSize = UI::Slider(1001, sld, currentSize, 0.10f, 1.25f, changed);
                y += 22.0f;
                char buf[128];
                snprintf(buf, sizeof(buf), "%.2f", currentSize);
                DrawText(buf, (int)x, (int)y, 16, Color{190, 190, 190, 255});
                y += 22.0f;
            }

            y += 6.0f;
            DrawText("Material", (int)x, (int)y, 18, RAYWHITE);
            y += 24.0f;

            {
                std::vector<std::string> matNames;
                matNames.reserve(presetMaterials.size() + 1);
                for (const auto& m : presetMaterials) matNames.push_back(m.name);
                matNames.push_back("Custom");

                int sel = currentMaterialIsCustom ? (int)presetMaterials.size() : -1;
                if (!currentMaterialIsCustom) {
                    for (int i = 0; i < (int)presetMaterials.size(); ++i) {
                        if (presetMaterials[i].name == currentMaterial.name) {
                            sel = i;
                            break;
                        }
                    }
                }
                if (sel == -1) sel = (int)presetMaterials.size(); // Default to custom if not found

                Rectangle r = {x, y, panel.width - 24.0f, 28.0f};
                matSelIndex = sel;
                UI::Dropdown(102, r, matNames, &matSelIndex);
                
                y += 34.0f;
            }

            auto sliderRow = [&](int idBase, const char* label, float& v, float minV, float maxV, int precision) {
                DrawText(label, (int)x, (int)y, 16, Color{200, 200, 200, 255});
                y += 18.0f;
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 14.0f};
                float nv = UI::Slider(idBase, sld, v, minV, maxV, changed);
                y += 18.0f;
                char buf[128];
                if (precision == 0) snprintf(buf, sizeof(buf), "%.0f", nv);
                else if (precision == 2) snprintf(buf, sizeof(buf), "%.2f", nv);
                else snprintf(buf, sizeof(buf), "%.3f", nv);
                DrawText(buf, (int)x, (int)y, 14, Color{170, 170, 170, 255});
                y += 18.0f;
                if (changed) {
                    v = nv;
                    currentMaterialIsCustom = true;
                }
            };

            sliderRow(1101, "Density", currentMaterial.density, 50.0f, 5000.0f, 0);
            sliderRow(1102, "Friction", currentMaterial.friction, 0.0f, 1.5f, 3);
            sliderRow(1103, "Restitution", currentMaterial.restitution, 0.0f, 1.0f, 3);

            y += 8.0f;
            DrawText("Floor", (int)x, (int)y, 18, RAYWHITE);
            y += 24.0f;

            {
                std::vector<std::string> matNames;
                matNames.reserve(presetMaterials.size() + 1);
                for (const auto& m : presetMaterials) matNames.push_back(m.name);
                matNames.push_back("Custom");

                int sel = floorMaterialIsCustom ? (int)presetMaterials.size() : -1;
                if (!floorMaterialIsCustom) {
                    for (int i = 0; i < (int)presetMaterials.size(); ++i) {
                        if (presetMaterials[i].name == floorMaterial.name) {
                            sel = i;
                            break;
                        }
                    }
                }
                if (sel == -1) sel = (int)presetMaterials.size();

                Rectangle r = {x, y, panel.width - 24.0f, 28.0f};
                
                floorSelIndex = sel;
                UI::Dropdown(103, r, matNames, &floorSelIndex);
                y += 34.0f;
            }

            auto floorSliderRow = [&](int idBase, const char* label, float& v, float minV, float maxV) {
                DrawText(label, (int)x, (int)y, 16, Color{200, 200, 200, 255});
                y += 18.0f;
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 14.0f};
                float nv = UI::Slider(idBase, sld, v, minV, maxV, changed);
                y += 18.0f;
                char buf[128];
                snprintf(buf, sizeof(buf), "%.3f", nv);
                DrawText(buf, (int)x, (int)y, 14, Color{170, 170, 170, 255});
                y += 18.0f;
                if (changed) {
                    v = nv;
                    floorMaterialIsCustom = true;
                }
            };
            floorSliderRow(1201, "Friction", floorMaterial.friction, 0.0f, 1.5f);
            floorSliderRow(1202, "Restitution", floorMaterial.restitution, 0.0f, 1.0f);

            contentHeight = (y + scrollY) - startY + pad;
            EndScissorMode();
            UI::setClipRect({0,0,0,0});

            Rectangle footer = {panel.x, panel.y + panel.height - footerH, panel.width, footerH};
            DrawRectangleRec(footer, Color{18, 18, 18, 245});
            DrawRectangleLinesEx(footer, 1.0f, Color{60, 60, 60, 255});
            
            if (isPaused) {
                DrawText("PAUSED", (int)(panel.x + pad), (int)(footer.y + 12.0f), 18, RED);
            } else {
                DrawText("Q Spawn | X Delete", (int)(panel.x + pad), (int)(footer.y + 12.0f), 18, RAYWHITE);
            }
            
            char timeBuf[32];
            snprintf(timeBuf, sizeof(timeBuf), "x%.1f", timeScale);
            DrawText(timeBuf, (int)(panel.x + panel.width - 60), (int)(footer.y + 12.0f), 18, RAYWHITE);

            int changedId = UI::DrawDropdownOverlay();
            if (changedId != -1) {
                if (changedId == 102) {
                    int idx = matSelIndex;
                    if (idx >= 0 && idx < (int)presetMaterials.size()) {
                        currentMaterial = presetMaterials[idx];
                        currentMaterialIsCustom = false;
                    } else {
                        currentMaterialIsCustom = true;
                    }
                } else if (changedId == 103) {
                    int idx = floorSelIndex;
                    if (idx >= 0 && idx < (int)presetMaterials.size()) {
                        floorMaterial = presetMaterials[idx];
                        floorMaterialIsCustom = false;
                    } else {
                        floorMaterialIsCustom = true;
                    }
                }
            }
        }

            int cx = GetScreenWidth() / 2;
            int cy = GetScreenHeight() / 2;
            DrawLine(cx - 8, cy, cx + 8, cy, Color{0,0,0,255});
            DrawLine(cx, cy - 8, cx, cy + 8, Color{0,0,0,255});
            DrawLine(cx - 4, cy, cx + 4, cy, Color{235,235,235,255});
            DrawLine(cx, cy - 4, cx, cy + 4, Color{235,235,235,255});

        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}