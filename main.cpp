#include "physics/PhysicsWorld.h"
#include <chrono>

float getDeltaTime() {
    static auto last = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> diff = now - last;
    last = now;
    return diff.count();
}

void render() {
    // TODO: rener scene
}

int main() {
    const float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;
    bool running = true;

    PhysicsWorld physicsWorld;

    while (running) {
        float frameTime = getDeltaTime();
        accumulator += frameTime;

        while (accumulator >= FIXED_DT) {
            physicsWorld.step(FIXED_DT);
            accumulator -= FIXED_DT;
        }

        render();

        if (accumulator > 1.0f) running = false;
    }

    return 0;
}