#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "render/Renderer.h"
#include <chrono>
#include <raylib.h>

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
    
    PhysicsWorld physicsWorld;

    RigidBody b1; b1.position = {0.0f, 3.0f, 0.0f}; b1.velocity = {0.0f, 0.0f, 0.0f}; b1.mass = 1.0f;
    RigidBody b2; b2.position = {1.0f, 8.0f, 3.0f}; b2.velocity = {0.0f, -100.0f, 0.0f}; b2.mass = 2.0f;
    physicsWorld.addRigidBody(&b1);
    physicsWorld.addRigidBody(&b2);

    Renderer renderer;
    if (!renderer.init(1280, 720, "Kinematica Sandbox")) return -1;

    while (!WindowShouldClose()) {
        float frameTime = getDeltaTime();
        accumulator += frameTime;

        while (accumulator >= FIXED_DT) {
            physicsWorld.step(FIXED_DT);
            accumulator -= FIXED_DT;
        }

        renderer.beginFrame();
        renderer.drawRigidBody(&b1, 0.5f);
        renderer.drawRigidBody(&b2, 0.5f);
        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}