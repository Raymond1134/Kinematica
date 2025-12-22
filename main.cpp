#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "physics/Spring.h"
#include "physics/collision/TriangleMesh.h"
#include "render/Renderer.h"
#include <chrono>
#include <raylib.h>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <list>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

struct MaterialProps {
    std::string name;
    float density = 1.0f;
    float friction = 0.5f;
    float restitution = 0.0f;
};

static std::string readFileText(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    std::string s;
    f.seekg(0, std::ios::end);
    s.resize((size_t)f.tellg());
    f.seekg(0, std::ios::beg);
    f.read(s.data(), (std::streamsize)s.size());
    return s;
}

static bool jsonFindString(const std::string& json, const char* key, std::string& out) {
    std::string k = std::string("\"") + key + "\"";
    size_t p = json.find(k);
    if (p == std::string::npos) return false;
    p = json.find(':', p + k.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
    if (p >= json.size() || json[p] != '"') return false;
    ++p;
    size_t e = p;
    while (e < json.size() && json[e] != '"') ++e;
    if (e >= json.size()) return false;
    out = json.substr(p, e - p);
    return true;
}

static bool jsonFindNumber(const std::string& json, const char* key, float& out) {
    std::string k = std::string("\"") + key + "\"";
    size_t p = json.find(k);
    if (p == std::string::npos) return false;
    p = json.find(':', p + k.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
    size_t e = p;
    while (e < json.size()) {
        char c = json[e];
        if (!(std::isdigit((unsigned char)c) || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E')) break;
        ++e;
    }
    if (e == p) return false;
    try {
        out = std::stof(json.substr(p, e - p));
        return true;
    } catch (...) {
        return false;
    }
}

static float getDeltaTime() {
    using clock = std::chrono::high_resolution_clock;
    static auto last = clock::now();
    auto now = clock::now();
    std::chrono::duration<float> dt = now - last;
    last = now;
    return dt.count();
}

static std::vector<MaterialProps> loadMaterialsFromFolder(const char* folder) {
    std::vector<MaterialProps> out;
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::exists(folder, ec) || !fs::is_directory(folder, ec)) return out;

    for (const auto& entry : fs::directory_iterator(folder, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != ".json") continue;

        std::string text = readFileText(entry.path().string());
        if (text.empty()) continue;

        MaterialProps m;
        if (!jsonFindString(text, "name", m.name)) {
            m.name = entry.path().stem().string();
        }
        jsonFindNumber(text, "density", m.density);
        jsonFindNumber(text, "friction", m.friction);
        jsonFindNumber(text, "restitution", m.restitution);

        if (m.density <= 0.0f) m.density = 1.0f;
        m.friction = std::clamp(m.friction, 0.0f, 2.0f);
        m.restitution = std::clamp(m.restitution, 0.0f, 1.0f);
        out.push_back(m);
    }

    std::sort(out.begin(), out.end(), [](const MaterialProps& a, const MaterialProps& b) { return a.name < b.name; });
    return out;
}

static float computePolyhedronVolume(const PolyhedronShape& poly) {
    if (poly.tris.empty() || poly.verts.empty()) return 0.0f;
    double sum = 0.0;
    for (const auto& t : poly.tris) {
        const Vec3& a = poly.verts[t.a];
        const Vec3& b = poly.verts[t.b];
        const Vec3& c = poly.verts[t.c];
        double v6 = (double)Vec3::dot(a, Vec3::cross(b, c));
        sum += std::fabs(v6);
    }
    double vol = sum / 6.0;
    return (float)vol;
}

enum class SpawnShape {
    Sphere,
    Cube,
    Capsule,
    Tetrahedron,
    Bipyramid,
    Dodecahedron,
    Ramp,
};

static std::shared_ptr<TriangleMesh> makeRampMesh(float width, float length, float height) {
    float w = std::max(0.01f, width);
    float l = std::max(0.01f, length);
    float h = std::max(0.0f, height);

    std::vector<Vec3> v;
    v.reserve(6);
    v.push_back({-0.5f * w, 0.0f, -0.5f * l});
    v.push_back({+0.5f * w, 0.0f, -0.5f * l});
    v.push_back({+0.5f * w, 0.0f, +0.5f * l});
    v.push_back({-0.5f * w, 0.0f, +0.5f * l});
    v.push_back({+0.5f * w, h,    +0.5f * l});
    v.push_back({-0.5f * w, h,    +0.5f * l});

    std::vector<uint32_t> idx;
    idx.reserve(10 * 3);
    idx.insert(idx.end(), {0, 4, 1, 0, 5, 4});
    idx.insert(idx.end(), {0, 3, 5});
    idx.insert(idx.end(), {1, 4, 2});
    idx.insert(idx.end(), {2, 4, 5, 2, 5, 3});
    idx.insert(idx.end(), {0, 1, 2, 0, 2, 3});

    auto m = std::make_shared<TriangleMesh>();
    m->build(v, idx);
    m->flags |= TriangleMesh::CollideUpwardOnly;
    return m;
}

static std::shared_ptr<TriangleMesh> makeRampCollisionMesh(float width, float length, float height) {
    float w = std::max(0.01f, width);
    float l = std::max(0.01f, length);
    float h = std::max(0.0f, height);

    std::vector<Vec3> v;
    v.reserve(4);
    v.push_back({-0.5f * w, 0.0f, -0.5f * l});
    v.push_back({+0.5f * w, 0.0f, -0.5f * l});
    v.push_back({+0.5f * w, h,    +0.5f * l});
    v.push_back({-0.5f * w, h,    +0.5f * l});

    std::vector<uint32_t> idx;
    idx.reserve(6);
    idx.insert(idx.end(), {0, 1, 2, 0, 2, 3});

    auto m = std::make_shared<TriangleMesh>();
    m->build(v, idx);
    return m;
}

static std::vector<Vec3> makeTetraVerts(float s) {
    return {
        {s, s, s},
        {-s, -s, s},
        {-s, s, -s},
        {s, -s, -s},
    };
}

static std::vector<Vec3> makePentagonalBipyramidVerts(float s) {
    float h = 1.0f * s;
    float r = 0.85f * s;
    std::vector<Vec3> v;
    v.reserve(7);
    v.push_back({0.0f, h, 0.0f});
    v.push_back({0.0f, -h, 0.0f});
    for (int i = 0; i < 5; ++i) {
        float a = (float)i * 2.0f * 3.14159265f / 5.0f;
        v.push_back({r * cosf(a), 0.0f, r * sinf(a)});
    }
    return v;
}

static std::vector<Vec3> makeDodecaVerts(float s) {
    const float phi = (1.0f + sqrtf(5.0f)) * 0.5f;
    const float a = 1.0f / phi;
    const float b = phi;

    std::vector<Vec3> v;
    v.reserve(20);

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                v.push_back({(float)sx, (float)sy, (float)sz});
            }
        }
    }

    for (int sy = -1; sy <= 1; sy += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            v.push_back({0.0f, (float)sy * a, (float)sz * b});
        }
    }
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            v.push_back({(float)sx * a, (float)sy * b, 0.0f});
        }
    }
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            v.push_back({(float)sx * b, 0.0f, (float)sz * a});
        }
    }

    for (Vec3& p : v) p = p * s;
    return v;
}

static bool uiRadio(Rectangle r, const char* text, bool selected) {
    Vector2 m = GetMousePosition();
    bool hot = CheckCollisionPointRec(m, r);
    Color bg = hot ? Color{45, 45, 45, 255} : Color{25, 25, 25, 255};
    DrawRectangleRec(r, bg);
    DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});

    Rectangle c = {r.x + 6, r.y + 7, 14, 14};
    DrawRectangleLinesEx(c, 1.0f, RAYWHITE);
    if (selected) {
        DrawRectangle((int)c.x + 3, (int)c.y + 3, (int)c.width - 6, (int)c.height - 6, RAYWHITE);
    }
    DrawText(text, (int)(r.x + 26), (int)(r.y + 5), 18, RAYWHITE);

    return hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

static float uiSlider(int id, Rectangle r, float value, float minV, float maxV, bool& changed) {
    Vector2 m = GetMousePosition();
    bool hot = CheckCollisionPointRec(m, r);
    static bool dragging = false;
    static int dragId = -1;

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && hot) {
        dragging = true;
        dragId = id;
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && dragging && dragId == id) {
        dragging = false;
        dragId = -1;
    }

    float t = (maxV > minV) ? ((value - minV) / (maxV - minV)) : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);

    DrawRectangleRec(r, Color{25, 25, 25, 255});
    DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});
    Rectangle fill = {r.x, r.y, r.width * t, r.height};
    DrawRectangleRec(fill, Color{60, 60, 60, 255});

    float knobX = r.x + r.width * t;
    DrawRectangle((int)(knobX - 3), (int)r.y - 2, 6, (int)r.height + 4, RAYWHITE);

    if (dragging && dragId == id) {
        float nt = (m.x - r.x) / r.width;
        nt = std::clamp(nt, 0.0f, 1.0f);
        float nv = minV + (maxV - minV) * nt;
        if (fabsf(nv - value) > 1e-6f) {
            value = nv;
            changed = true;
        }
    }
    return value;
}

int main() {
    const float FIXED_DT = 1.0f / 60.0f;
    float accumulator = 0.0f;

    const int MAX_SUBSTEPS = 4;
    
    PhysicsWorld physicsWorld;
    std::list<RigidBody> dynamicBodies;

    std::vector<MaterialProps> presetMaterials = loadMaterialsFromFolder("materials");
    MaterialProps customTemplate;
    customTemplate.name = "Custom";

    MaterialProps currentMaterial = presetMaterials.empty() ? customTemplate : presetMaterials[0];
    bool currentMaterialIsCustom = presetMaterials.empty();

    MaterialProps floorMaterial = presetMaterials.empty() ? customTemplate : presetMaterials[0];
    bool floorMaterialIsCustom = presetMaterials.empty();

    SpawnShape currentShape = SpawnShape::Cube;
    float currentSize = 0.35f;


    Renderer renderer;
    if (!renderer.init(1280, 720, "Kinematica Sandbox")) return -1;
    {
        constexpr int wallW = 10;
        constexpr int wallH = 10;
        const Vec3 half = {0.25f, 0.25f, 0.25f};
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
            ToggleFullscreen();
            if (!IsWindowFullscreen()) {
                SetWindowSize(1280, 720);
                int mon = GetCurrentMonitor();
                int mw = GetMonitorWidth(mon);
                int mh = GetMonitorHeight(mon);
                SetWindowPosition((mw - 1280) / 2, (mh - 720) / 2);
            }
        }
        float frameTime = getDeltaTime();
        if (frameTime > 0.25f) frameTime = 0.25f;
        accumulator += frameTime;

        physicsWorld.floorFriction = std::clamp(floorMaterial.friction, 0.0f, 2.0f);
        physicsWorld.floorRestitution = std::clamp(floorMaterial.restitution, 0.0f, 1.0f);

        if (IsKeyPressed(KEY_Q)) {
            const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();

            Vec3 up = {0.0f, 1.0f, 0.0f};
            Vec3 right = Vec3::cross(up, camFwd).normalized();
            if (right.lengthSq() < 1e-8f) right = {1.0f, 0.0f, 0.0f};

            RigidBody b;
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

            b.isStatic = false;
            b.sleeping = false;
            b.sleepTimer = 0.0f;

            b.friction = std::clamp(currentMaterial.friction, 0.0f, 2.0f);
            b.restitution = std::clamp(currentMaterial.restitution, 0.0f, 1.0f);

            float volume = 0.0f;
            float s = std::clamp(currentSize, 0.10f, 1.25f);
            switch (currentShape) {
                case SpawnShape::Sphere: {
                    b.collider = Collider::createSphere(s);
                    volume = (4.0f / 3.0f) * 3.14159265f * s * s * s;
                    break;
                }
                case SpawnShape::Cube: {
                    b.collider = Collider::createBox({s, s, s});
                    volume = (2.0f * s) * (2.0f * s) * (2.0f * s);
                    break;
                }
                case SpawnShape::Capsule: {
                    float r = s;
                    float hh = 1.5f * s;
                    b.collider = Collider::createCapsule(r, hh);
                    float cylH = 2.0f * hh;
                    volume = 3.14159265f * r * r * cylH + (4.0f / 3.0f) * 3.14159265f * r * r * r;
                    break;
                }
                case SpawnShape::Tetrahedron: {
                    auto verts = makeTetraVerts(s);
                    b.collider = Collider::createConvex(verts);
                    volume = b.collider.polyhedron ? computePolyhedronVolume(*b.collider.polyhedron) : 0.0f;
                    break;
                }
                case SpawnShape::Bipyramid: {
                    auto verts = makePentagonalBipyramidVerts(s);
                    b.collider = Collider::createConvex(verts);
                    volume = b.collider.polyhedron ? computePolyhedronVolume(*b.collider.polyhedron) : 0.0f;
                    break;
                }
                case SpawnShape::Dodecahedron: {
                    auto verts = makeDodecaVerts(s);
                    b.collider = Collider::createConvex(verts);
                    volume = b.collider.polyhedron ? computePolyhedronVolume(*b.collider.polyhedron) : 0.0f;
                    break;
                }
                case SpawnShape::Ramp: {
                    float width = 2.5f * s;
                    float length = 4.0f * s;
                    float height = 1.4f * s;
                    auto rampRender = makeRampMesh(width, length, height);
                    auto rampCollision = shiftDown ? rampRender : makeRampCollisionMesh(width, length, height);
                    b.collider = Collider::createMesh(rampCollision, rampRender);

                    Vec3 f = {camFwd.x, 0.0f, camFwd.z};
                    if (f.lengthSq() < 1e-8f) f = {0.0f, 0.0f, 1.0f};
                    f = f.normalized();
                    float yaw = atan2f(f.x, f.z);
                    Quat qYaw = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, yaw);
                    b.orientation = qYaw.normalized();

                    b.position = {camPos.x + f.x * 4.0f, physicsWorld.floorY + 0.001f, camPos.z + f.z * 4.0f};
                    b.velocity = {0.0f, 0.0f, 0.0f};
                    b.angularVelocity = {0.0f, 0.0f, 0.0f};
                    b.isStatic = !shiftDown;
                    b.sleeping = !shiftDown;
                    b.sleepTimer = shiftDown ? 0.0f : 1.0f;
                    if (shiftDown) {
                        b.position.y = physicsWorld.floorY + height + 1.0f;
                    }

                    volume = width * length * std::max(0.01f, height) * 0.5f;
                    break;
                }
            }

            float density = std::max(1.0f, currentMaterial.density);
            b.mass = std::max(0.05f, density * std::max(0.0f, volume));

            dynamicBodies.push_back(b);
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
        renderer.end3D();
        {
            static float uiScroll = 0.0f;
            int sw = GetScreenWidth();
            int sh = GetScreenHeight();

            float panelW = std::min(360.0f, std::max(240.0f, sw * 0.28f));
            panelW = std::min(panelW, (float)sw);

            Rectangle panel = {(float)sw - panelW, 0.0f, panelW, (float)sh};
            renderer.setUiBlockRect(panel);

            DrawRectangleRec(panel, Color{18, 18, 18, 235});
            DrawRectangleLinesEx(panel, 1.0f, Color{60, 60, 60, 255});

            const float pad = 12.0f;
            const float footerH = 42.0f;
            Rectangle contentClip = {panel.x + 1.0f, panel.y + 1.0f, panel.width - 2.0f, std::max(0.0f, panel.height - footerH - 2.0f)};

            bool mouseOverPanel = CheckCollisionPointRec(GetMousePosition(), panel);
            if (mouseOverPanel) {
                float wheel = GetMouseWheelMove();
                if (wheel != 0.0f) uiScroll += wheel * 28.0f;
            }

            if (uiScroll > 0.0f) uiScroll = 0.0f;

            BeginScissorMode((int)contentClip.x, (int)contentClip.y, (int)contentClip.width, (int)contentClip.height);

            float x = panel.x + pad;
            float y = pad + uiScroll;
            DrawText("Spawner", (int)x, (int)y, 22, RAYWHITE);
            y += 34.0f;

            DrawText("Shape", (int)x, (int)y, 18, RAYWHITE);
            y += 24.0f;
            auto shapeRadio = [&](SpawnShape s, const char* name) {
                Rectangle r = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r, name, currentShape == s)) currentShape = s;
                y += 28.0f;
            };
            shapeRadio(SpawnShape::Sphere, "Sphere");
            shapeRadio(SpawnShape::Cube, "Cube");
            shapeRadio(SpawnShape::Capsule, "Capsule");
            shapeRadio(SpawnShape::Tetrahedron, "Tetrahedron");
            shapeRadio(SpawnShape::Bipyramid, "Bipyramid");
            shapeRadio(SpawnShape::Dodecahedron, "Dodecahedron");
            shapeRadio(SpawnShape::Ramp, "Ramp");

            y += 8.0f;
            DrawText("Size", (int)x, (int)y, 18, RAYWHITE);
            y += 22.0f;
            {
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 16.0f};
                currentSize = uiSlider(1001, sld, currentSize, 0.10f, 1.25f, changed);
                y += 22.0f;
                char buf[128];
                snprintf(buf, sizeof(buf), "%.2f", currentSize);
                DrawText(buf, (int)x, (int)y, 16, Color{190, 190, 190, 255});
                y += 22.0f;
            }

            y += 6.0f;
            DrawText("Material", (int)x, (int)y, 18, RAYWHITE);
            y += 24.0f;

            auto materialRadio = [&](const MaterialProps& m) {
                bool sel = (!currentMaterialIsCustom && currentMaterial.name == m.name);
                Rectangle r = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r, m.name.c_str(), sel)) {
                    currentMaterial = m;
                    currentMaterialIsCustom = false;
                }
                y += 28.0f;
            };
            for (const auto& m : presetMaterials) materialRadio(m);
            {
                Rectangle r = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r, "Custom", currentMaterialIsCustom)) {
                    currentMaterialIsCustom = true;
                }
                y += 32.0f;
            }

            auto sliderRow = [&](int idBase, const char* label, float& v, float minV, float maxV, int precision) {
                DrawText(label, (int)x, (int)y, 16, Color{200, 200, 200, 255});
                y += 18.0f;
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 14.0f};
                float nv = uiSlider(idBase, sld, v, minV, maxV, changed);
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

            auto floorMaterialRadio = [&](const MaterialProps& m) {
                bool sel = (!floorMaterialIsCustom && floorMaterial.name == m.name);
                Rectangle r = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r, m.name.c_str(), sel)) {
                    floorMaterial = m;
                    floorMaterialIsCustom = false;
                }
                y += 28.0f;
            };
            for (const auto& m : presetMaterials) floorMaterialRadio(m);
            {
                Rectangle r = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r, "Custom", floorMaterialIsCustom)) {
                    floorMaterialIsCustom = true;
                }
                y += 32.0f;
            }

            auto floorSliderRow = [&](int idBase, const char* label, float& v, float minV, float maxV) {
                DrawText(label, (int)x, (int)y, 16, Color{200, 200, 200, 255});
                y += 18.0f;
                bool changed = false;
                Rectangle sld = {x, y, panel.width - 24.0f, 14.0f};
                float nv = uiSlider(idBase, sld, v, minV, maxV, changed);
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

            float contentBottomNoScroll = y - uiScroll;
            float visibleH = panel.height - footerH - pad;
            float contentH = contentBottomNoScroll - pad;
            float minScroll = std::min(0.0f, visibleH - contentH);
            uiScroll = std::clamp(uiScroll, minScroll, 0.0f);

            EndScissorMode();

            Rectangle footer = {panel.x, panel.y + panel.height - footerH, panel.width, footerH};
            DrawRectangleRec(footer, Color{18, 18, 18, 245});
            DrawRectangleLinesEx(footer, 1.0f, Color{60, 60, 60, 255});
            DrawText("Q spawn | Shift+Q: dynamic ramp (mesh-floor)", (int)(panel.x + pad), (int)(footer.y + 12.0f), 18, RAYWHITE);
        }

        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}