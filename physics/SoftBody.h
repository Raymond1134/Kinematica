#pragma once
#include <vector>
#include <raylib.h>
#include "../math/Vec3.h"

class RigidBody;
class Renderer;

struct SoftBody {
    std::vector<RigidBody*> particles;
    int rows = 0;
    int cols = 0;
    int layers = 0;
    bool isBlob = false;
    Mesh mesh = { 0 };
    Model model = { 0 };
    Color color = WHITE;
    Vec3 boundsCenter = {0,0,0};
    float boundsRadius = 0.0f;

    void init(int r, int c, Color col);
    void initBlob(int r, int c, int l, Color col);
    void update();
    void draw(const Renderer& renderer);
};
