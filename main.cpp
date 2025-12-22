#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "physics/Spring.h"
#include "render/Renderer.h"
#include <chrono>
#include <raylib.h>
#include <list>

static float getDeltaTime() {
    static auto last = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> diff = now - last;
    last = now;
    return diff.count();
}

int main() {
    const float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;

    const int MAX_SUBSTEPS = 4;
    
    PhysicsWorld physicsWorld;
    std::list<RigidBody> dynamicBodies;


    Renderer renderer;
    if (!renderer.init(1280, 720, "Kinematica Sandbox")) return -1;
    {
        constexpr int wallW = 12;
        constexpr int wallH = 12;
        const Vec3 half = {0.15f, 0.15f, 0.15f};
        const float spacingX = half.x * 2.0f;
        const float spacingY = half.y * 2.0f;

        const float baseX = -0.5f * (wallW - 1) * spacingX;
        const float baseY = physicsWorld.floorY + half.y;
        const float z = 0.0f;

        for (int y = 0; y < wallH; ++y) {
            for (int x = 0; x < wallW; ++x) {
                RigidBody b;
                b.position = {baseX + x * spacingX, baseY + y * spacingY, z};
                b.velocity = {0.0f, 0.0f, 0.0f};
                b.angularVelocity = {0.0f, 0.0f, 0.0f};
                b.orientation = Quat::identity();
                b.mass = 1.0f;
                b.friction = 0.65f;
                b.restitution = 0.0f;
                b.collider = Collider::createBox(half);
                b.isStatic = false;
                b.sleeping = true;
                b.sleepTimer = 1.0f;

                dynamicBodies.push_back(b);
                physicsWorld.addRigidBody(&dynamicBodies.back());
            }
        }
    }

    SetExitKey(0);

    while (!WindowShouldClose()) {
        if ((IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) && IsKeyPressed(KEY_ENTER)) {
            if (IsWindowFullscreen()) CloseWindow(), InitWindow(1280, 720, "Kinematica Sandbox");
            else ToggleFullscreen();
        }
        float frameTime = getDeltaTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        accumulator += frameTime;

        if (IsKeyPressed(KEY_ONE)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            
            RigidBody newSphere;
            newSphere.position = camPos + camFwd * 2.0f;
            newSphere.velocity = camFwd * 10.0f;
            newSphere.angularVelocity = {0.0f, 0.0f, 0.0f};
            newSphere.orientation = Quat::identity();
            newSphere.mass = 25.0f;
            newSphere.friction = 0.40f;
            newSphere.restitution = 0.26f;
            newSphere.collider = Collider::createSphere(0.5f);
            dynamicBodies.push_back(newSphere);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        
        if (IsKeyPressed(KEY_TWO)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            
            RigidBody newBox;
            newBox.position = camPos + camFwd * 2.0f;
            newBox.velocity = camFwd * 10.0f;
            Vec3 up = {0.0f, 1.0f, 0.0f};
            Vec3 right = Vec3::cross(up, camFwd).normalized();
            if (right.lengthSq() < 1e-8f) right = {1.0f, 0.0f, 0.0f};

            float tiltDeg = (float)GetRandomValue(-5, 5);
            float rollDeg = (float)GetRandomValue(-5, 5);
            Quat qTilt = Quat::fromAxisAngle(right, tiltDeg * DEG2RAD);
            Quat qRoll = Quat::fromAxisAngle(camFwd, rollDeg * DEG2RAD);
            newBox.orientation = (qRoll * qTilt).normalized();

            float spin = 7.0f;
            float spinJitter = (float)GetRandomValue(-5, 5) / 5.0f;
            newBox.angularVelocity = right * (spin * (1.0f + 0.25f * spinJitter)) + up * (1.0f * spinJitter);
            newBox.mass = 3.0f;
            newBox.friction = 0.60f;
            newBox.restitution = 0.035f;
            newBox.collider = Collider::createBox({0.4f, 0.4f, 0.4f});
            dynamicBodies.push_back(newBox);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        
        if (IsKeyPressed(KEY_THREE)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            
            RigidBody newCapsule;
            newCapsule.position = camPos + camFwd * 2.0f;
            newCapsule.velocity = camFwd * 10.0f;
            newCapsule.angularVelocity = {0.0f, 0.0f, 0.0f};
            newCapsule.orientation = Quat::identity();
            newCapsule.mass = 1.2f;
            newCapsule.friction = 0.25f;
            newCapsule.restitution = 0.22f;
            newCapsule.collider = Collider::createCapsule(0.3f, 0.5f);
            dynamicBodies.push_back(newCapsule);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }

        if (IsKeyPressed(KEY_FOUR)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            // Tetrahedron (regular)
            float s = 0.4f;
            std::vector<Vec3> verts = {
                {s, s, s},
                {-s, -s, s},
                {-s, s, -s},
                {s, -s, -s}
            };
            RigidBody newConvex;
            newConvex.position = camPos + camFwd * 2.0f;
            newConvex.velocity = camFwd * 10.0f;
            newConvex.angularVelocity = {0.0f, 0.0f, 0.0f};
            Vec3 up = {0.0f, 1.0f, 0.0f};
            newConvex.orientation = Quat::lookRotation(camFwd, up);
            newConvex.mass = 1.0f;
            newConvex.friction = 0.35f;
            newConvex.restitution = 0.15f;
            newConvex.collider = Collider::createConvex(verts);
            dynamicBodies.push_back(newConvex);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        if (IsKeyPressed(KEY_FIVE)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            // Trigonal bipyramid (max separation)
            float h = 0.4f;
            float r = 0.3f;
            std::vector<Vec3> verts = {
                {0, h, 0},
                {0, -h, 0},
                {r, 0, 0},
                {-r/2, 0, r * sqrtf(3)/2},
                {-r/2, 0, -r * sqrtf(3)/2}
            };
            RigidBody newConvex;
            newConvex.position = camPos + camFwd * 2.0f;
            newConvex.velocity = camFwd * 10.0f;
            newConvex.angularVelocity = {0.0f, 0.0f, 0.0f};
            Vec3 up = {0.0f, 1.0f, 0.0f};
            newConvex.orientation = Quat::lookRotation(camFwd, up);
            newConvex.mass = 1.0f;
            newConvex.friction = 0.35f;
            newConvex.restitution = 0.15f;
            newConvex.collider = Collider::createConvex(verts);
            dynamicBodies.push_back(newConvex);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        if (IsKeyPressed(KEY_SIX)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            // Octahedron (regular)
            float s = 0.4f;
            std::vector<Vec3> verts = {
                {s, 0, 0},
                {-s, 0, 0},
                {0, s, 0},
                {0, -s, 0},
                {0, 0, s},
                {0, 0, -s}
            };
            RigidBody newConvex;
            newConvex.position = camPos + camFwd * 2.0f;
            newConvex.velocity = camFwd * 10.0f;
            newConvex.angularVelocity = {0.0f, 0.0f, 0.0f};
            Vec3 up = {0.0f, 1.0f, 0.0f};
            newConvex.orientation = Quat::lookRotation(camFwd, up);
            newConvex.mass = 1.0f;
            newConvex.friction = 0.35f;
            newConvex.restitution = 0.15f;
            newConvex.collider = Collider::createConvex(verts);
            dynamicBodies.push_back(newConvex);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        if (IsKeyPressed(KEY_SEVEN)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            // Pentagonal bipyramid
            float h = 0.35f;
            float r = 0.25f;
            std::vector<Vec3> verts = {
                {0, h, 0},
                {0, -h, 0},
            };
            for (int i = 0; i < 5; ++i) {
                float a = i * 2 * 3.14159265f / 5;
                verts.push_back({r * cosf(a), 0, r * sinf(a)});
            }
            RigidBody newConvex;
            newConvex.position = camPos + camFwd * 2.0f;
            newConvex.velocity = camFwd * 10.0f;
            newConvex.angularVelocity = {0.0f, 0.0f, 0.0f};
            Vec3 up = {0.0f, 1.0f, 0.0f};
            newConvex.orientation = Quat::lookRotation(camFwd, up);
            newConvex.mass = 1.0f;
            newConvex.friction = 0.35f;
            newConvex.restitution = 0.15f;
            newConvex.collider = Collider::createConvex(verts);
            dynamicBodies.push_back(newConvex);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        if (IsKeyPressed(KEY_EIGHT)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            // Cube corners
            float s = 0.35f;
            std::vector<Vec3> verts = {
                {s, s, s},
                {s, s, -s},
                {s, -s, s},
                {s, -s, -s},
                {-s, s, s},
                {-s, s, -s},
                {-s, -s, s},
                {-s, -s, -s}
            };
            RigidBody newConvex;
            newConvex.position = camPos + camFwd * 2.0f;
            newConvex.velocity = camFwd * 10.0f;
            newConvex.angularVelocity = {0.0f, 0.0f, 0.0f};
            Vec3 up = {0.0f, 1.0f, 0.0f};
            newConvex.orientation = Quat::lookRotation(camFwd, up);
            newConvex.mass = 0.1f;
            newConvex.friction = 0.35f;
            newConvex.restitution = 0.0f;
            newConvex.collider = Collider::createConvex(verts);
            dynamicBodies.push_back(newConvex);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }
        if (IsKeyPressed(KEY_NINE)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            // 9 points on a sphere
            float r = 0.4f;
            std::vector<Vec3> verts;
            for (int i = 0; i < 9; ++i) {
                float phi = acosf(-1.0f + 2.0f * (i + 1) / 10.0f);
                float theta = 3.14159265f * (1 + sqrtf(5)) * i;
                verts.push_back({r * sinf(phi) * cosf(theta), r * sinf(phi) * sinf(theta), r * cosf(phi)});
            }
            RigidBody newConvex;
            newConvex.position = camPos + camFwd * 2.0f;
            newConvex.velocity = camFwd * 10.0f;
            newConvex.angularVelocity = {0.0f, 0.0f, 0.0f};
            Vec3 up = {0.0f, 1.0f, 0.0f};
            newConvex.orientation = Quat::lookRotation(camFwd, up);
            newConvex.mass = 1.0f;
            newConvex.friction = 0.35f;
            newConvex.restitution = 0.15f;
            newConvex.collider = Collider::createConvex(verts);
            dynamicBodies.push_back(newConvex);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }

        if (IsKeyPressed(KEY_ZERO)) {
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            
            // U-shape made from 3 boxes.
            std::vector<CompoundChild> children;
            {
                CompoundChild base;
                base.collider = Collider::createBox({0.60f, 0.15f, 0.20f});
                base.localPosition = {0.0f, 0.0f, 0.0f};
                base.localOrientation = Quat::identity();
                children.push_back(base);

                CompoundChild left;
                left.collider = Collider::createBox({0.15f, 0.45f, 0.20f});
                left.localPosition = {-0.45f, 0.45f, 0.0f};
                left.localOrientation = Quat::identity();
                children.push_back(left);

                CompoundChild right;
                right.collider = Collider::createBox({0.15f, 0.45f, 0.20f});
                right.localPosition = {0.45f, 0.45f, 0.0f};
                right.localOrientation = Quat::identity();
                children.push_back(right);
            }

            RigidBody newCompound;
            newCompound.position = camPos + camFwd * 2.0f;
            newCompound.velocity = camFwd * 10.0f;

            Vec3 up = {0.0f, 1.0f, 0.0f};
            Vec3 rightAxis = Vec3::cross(up, camFwd).normalized();
            if (rightAxis.lengthSq() < 1e-8f) rightAxis = {1.0f, 0.0f, 0.0f};
            float tiltDeg = (float)GetRandomValue(-5, 5);
            float rollDeg = (float)GetRandomValue(-5, 5);
            Quat qTilt = Quat::fromAxisAngle(rightAxis, tiltDeg * DEG2RAD);
            Quat qRoll = Quat::fromAxisAngle(camFwd, rollDeg * DEG2RAD);
            newCompound.orientation = (qRoll * qTilt).normalized();

            newCompound.angularVelocity = {0.0f, 0.0f, 0.0f};
            newCompound.mass = 4.0f;
            newCompound.friction = 0.60f;
            newCompound.restitution = 0.03f;
            newCompound.collider = Collider::createCompound(children);

            dynamicBodies.push_back(newCompound);
            physicsWorld.addRigidBody(&dynamicBodies.back());
        }

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
        for (auto& body : dynamicBodies) {
            renderer.drawRigidBody(&body);
        }
        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}