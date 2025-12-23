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
#include <unordered_map>

struct MaterialProps {
    std::string name;
    float density = 1.0f;
    float friction = 0.5f;
    float restitution = 0.0f;

    Color color = Color{200, 200, 200, 255};
    float opacity = 1.0f;
    bool outline = false;
};

static bool jsonFindBool(const std::string& json, const char* key, bool& out) {
    std::string k = std::string("\"") + key + "\"";
    size_t p = json.find(k);
    if (p == std::string::npos) return false;
    p = json.find(':', p + k.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
    if (p + 3 < json.size() && json.compare(p, 4, "true") == 0) { out = true; return true; }
    if (p + 4 < json.size() && json.compare(p, 5, "false") == 0) { out = false; return true; }
    return false;
}

static bool parseHexByte(const std::string& s, size_t pos, unsigned char& out) {
    if (pos + 2 > s.size()) return false;
    auto hex = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        return -1;
    };
    int a = hex(s[pos]);
    int b = hex(s[pos + 1]);
    if (a < 0 || b < 0) return false;
    out = (unsigned char)((a << 4) | b);
    return true;
}

static bool jsonFindColor(const std::string& json, const char* key, Color& out) {
    std::string k = std::string("\"") + key + "\"";
    size_t p = json.find(k);
    if (p == std::string::npos) return false;
    p = json.find(':', p + k.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
    if (p >= json.size()) return false;

    // Support either:
    //   "color": [r,g,b]  (0-1 or 0-255)
    //   "color": "#RRGGBB"
    if (json[p] == '"') {
        ++p;
        size_t e = p;
        while (e < json.size() && json[e] != '"') ++e;
        if (e >= json.size()) return false;
        std::string s = json.substr(p, e - p);
        if (s.size() == 7 && s[0] == '#') {
            unsigned char r, g, b;
            if (!parseHexByte(s, 1, r) || !parseHexByte(s, 3, g) || !parseHexByte(s, 5, b)) return false;
            out = Color{r, g, b, 255};
            return true;
        }
        return false;
    }

    if (json[p] != '[') return false;
    ++p;
    float c[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
        size_t e = p;
        while (e < json.size()) {
            char ch = json[e];
            if (!(std::isdigit((unsigned char)ch) || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E')) break;
            ++e;
        }
        if (e == p) return false;
        try {
            c[i] = std::stof(json.substr(p, e - p));
        } catch (...) {
            return false;
        }
        p = e;
        while (p < json.size() && std::isspace((unsigned char)json[p])) ++p;
        if (i < 2) {
            if (p >= json.size() || json[p] != ',') return false;
            ++p;
        }
    }

    while (p < json.size() && json[p] != ']') ++p;
    if (p >= json.size()) return false;

    bool is255 = (c[0] > 1.0f || c[1] > 1.0f || c[2] > 1.0f);
    auto toByte = [&](float v) -> unsigned char {
        if (!std::isfinite(v)) v = 0.0f;
        if (is255) v = std::clamp(v, 0.0f, 255.0f);
        else v = std::clamp(v, 0.0f, 1.0f) * 255.0f;
        return (unsigned char)std::lround(v);
    };
    out = Color{toByte(c[0]), toByte(c[1]), toByte(c[2]), 255};
    return true;
}

static Color applyOpacity(Color c, float opacity) {
    float o = std::isfinite(opacity) ? opacity : 1.0f;
    o = std::clamp(o, 0.0f, 1.0f);
    c.a = (unsigned char)std::lround(o * 255.0f);
    return c;
}

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
        jsonFindNumber(text, "opacity", m.opacity);
        jsonFindBool(text, "outline", m.outline);
        jsonFindColor(text, "color", m.color);

        if (m.density <= 0.0f) m.density = 1.0f;
        m.friction = std::clamp(m.friction, 0.0f, 2.0f);
        m.restitution = std::clamp(m.restitution, 0.0f, 1.0f);
        m.opacity = std::clamp(m.opacity, 0.0f, 1.0f);
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

enum class SpawnMode {
    Throw,
    PlaceDynamic,
    PlaceStatic,
};

struct RayHit {
    bool hit = false;
    float t = 0.0f;
    Vec3 point = {0, 0, 0};
    Vec3 normal = {0, 1, 0};
};

static bool raySphere(const Vec3& ro, const Vec3& rd, const Vec3& c, float r, float& outT, Vec3& outN) {
    Vec3 oc = ro - c;
    float b = Vec3::dot(oc, rd);
    float cterm = Vec3::dot(oc, oc) - r * r;
    float disc = b * b - cterm;
    if (disc < 0.0f) return false;
    float s = std::sqrt(disc);
    float t0 = -b - s;
    float t1 = -b + s;
    float t = (t0 > 1e-5f) ? t0 : ((t1 > 1e-5f) ? t1 : -1.0f);
    if (t <= 0.0f) return false;
    Vec3 p = ro + rd * t;
    outN = (p - c).normalized();
    outT = t;
    return true;
}

static bool rayObbBox(const Vec3& ro, const Vec3& rd, const Vec3& c, const Quat& q, const Vec3& he, float& outT, Vec3& outN) {
    Vec3 roL = q.rotateInv(ro - c);
    Vec3 rdL = q.rotateInv(rd);

    float tmin = -1e30f;
    float tmax = 1e30f;
    Vec3 nEnter = {0.0f, 0.0f, 0.0f};

    auto slab = [&](float roA, float rdA, float minA, float maxA, const Vec3& nNeg, const Vec3& nPos) -> bool {
        if (std::fabs(rdA) < 1e-8f) {
            return (roA >= minA && roA <= maxA);
        }
        float inv = 1.0f / rdA;
        float t0 = (minA - roA) * inv;
        float t1 = (maxA - roA) * inv;
        Vec3 n0 = nNeg;
        Vec3 n1 = nPos;
        if (t0 > t1) { std::swap(t0, t1); std::swap(n0, n1); }
        if (t0 > tmin) { tmin = t0; nEnter = n0; }
        if (t1 < tmax) { tmax = t1; }
        return tmin <= tmax;
    };

    if (!slab(roL.x, rdL.x, -he.x, he.x, Vec3{-1, 0, 0}, Vec3{1, 0, 0})) return false;
    if (!slab(roL.y, rdL.y, -he.y, he.y, Vec3{0, -1, 0}, Vec3{0, 1, 0})) return false;
    if (!slab(roL.z, rdL.z, -he.z, he.z, Vec3{0, 0, -1}, Vec3{0, 0, 1})) return false;

    float t = (tmin > 1e-5f) ? tmin : tmax;
    if (t <= 1e-5f) return false;
    outT = t;
    outN = q.rotate(nEnter).normalized();
    return true;
}

static bool rayTri(const Vec3& ro, const Vec3& rd, const Vec3& a, const Vec3& b, const Vec3& c, float& outT, Vec3& outN) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 pvec = Vec3::cross(rd, ac);
    float det = Vec3::dot(ab, pvec);
    if (std::fabs(det) < 1e-8f) return false;
    float invDet = 1.0f / det;
    Vec3 tvec = ro - a;
    float u = Vec3::dot(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return false;
    Vec3 qvec = Vec3::cross(tvec, ab);
    float v = Vec3::dot(rd, qvec) * invDet;
    if (v < 0.0f || (u + v) > 1.0f) return false;
    float t = Vec3::dot(ac, qvec) * invDet;
    if (t <= 1e-5f) return false;

    Vec3 n = Vec3::cross(ab, ac).normalized();
    if (Vec3::dot(n, rd) > 0.0f) n = -n;
    outT = t;
    outN = n;
    return true;
}

static bool rayMesh(const Vec3& roW, const Vec3& rdW, const Vec3& posW, const Quat& qW, const TriangleMesh& mesh, float& outT, Vec3& outN) {
    float tS;
    Vec3 nS;
    if (!raySphere(roW, rdW, posW + qW.rotate(mesh.localCenter()), mesh.boundRadius, tS, nS)) {
        return false;
    }

    Vec3 ro = qW.rotateInv(roW - posW);
    Vec3 rd = qW.rotateInv(rdW);

    float bestT = 1e30f;
    Vec3 bestN = {0, 1, 0};

    for (const auto& t : mesh.tris) {
        if (t.a >= mesh.vertices.size() || t.b >= mesh.vertices.size() || t.c >= mesh.vertices.size()) continue;
        const Vec3& a = mesh.vertices[t.a];
        const Vec3& b = mesh.vertices[t.b];
        const Vec3& c = mesh.vertices[t.c];
        float tt;
        Vec3 nn;
        if (!rayTri(ro, rd, a, b, c, tt, nn)) continue;
        if (tt < bestT) { bestT = tt; bestN = nn; }
    }

    if (bestT >= 1e29f) return false;
    outT = bestT;
    outN = qW.rotate(bestN).normalized();
    return true;
}

static float placementOffsetAlongNormal(const Collider& c, const Quat& q, const Vec3& nWorld) {
    Vec3 n = nWorld.normalized();
    if (c.type == ColliderType::Mesh) {
        const TriangleMesh* meshPtr = c.renderMesh ? c.renderMesh.get() : c.mesh.get();
        if (!meshPtr || meshPtr->vertices.empty()) {
            return std::max(0.01f, c.boundingRadius());
        }
        float minD = +1e30f;
        for (const Vec3& vLocal : meshPtr->vertices) {
            Vec3 vWorld = q.rotate(vLocal);
            float d = Vec3::dot(vWorld, n);
            if (d < minD) minD = d;
        }
        if (!std::isfinite(minD)) return std::max(0.01f, c.boundingRadius());
        float offset = -minD;
        if (!std::isfinite(offset)) offset = c.boundingRadius();
        return std::max(0.01f, offset);
    }
    Vec3 nLocal = q.rotateInv(n);
    Vec3 pLocal = c.support(nLocal);
    Vec3 pWorld = q.rotate(pLocal);
    float d = Vec3::dot(pWorld, n);
    if (!std::isfinite(d)) d = c.boundingRadius();
    return std::max(0.01f, d);
}

static RayHit raycastWorldPlacement(const Vec3& ro, const Vec3& rd, float floorY, const std::list<RigidBody>& bodies) {
    RayHit best;
    best.hit = false;
    best.t = 1e30f;

    if (std::fabs(rd.y) > 1e-8f) {
        float t = (floorY - ro.y) / rd.y;
        if (t > 1e-5f && t < best.t) {
            best.hit = true;
            best.t = t;
            best.point = ro + rd * t;
            best.normal = {0.0f, 1.0f, 0.0f};
        }
    }

    for (const RigidBody& b : bodies) {
        const Collider& c = b.collider;

        float tS;
        Vec3 nS;
        float br = c.boundingRadius();
        if (br > 0.0f) {
            if (!raySphere(ro, rd, b.position, br, tS, nS)) {
                continue;
            }
        }

        float t = 0.0f;
        Vec3 n = {0.0f, 1.0f, 0.0f};
        bool hit = false;

        switch (c.type) {
            case ColliderType::Sphere:
                hit = raySphere(ro, rd, b.position, c.sphere.radius, t, n);
                break;
            case ColliderType::Box:
                hit = rayObbBox(ro, rd, b.position, b.orientation, c.box.halfExtents, t, n);
                break;
            case ColliderType::Capsule:
            case ColliderType::Convex:
            case ColliderType::Compound:
                hit = raySphere(ro, rd, b.position, br, t, n);
                break;
            case ColliderType::Mesh:
                if (c.mesh) {
                    const TriangleMesh& mesh = c.renderMesh ? *c.renderMesh : *c.mesh;
                    hit = rayMesh(ro, rd, b.position, b.orientation, mesh, t, n);
                }
                break;
        }

        if (hit && t > 1e-5f && t < best.t) {
            best.hit = true;
            best.t = t;
            best.point = ro + rd * t;
            best.normal = n;
        }
    }

    return best;
}

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
    const float invPhi = 1.0f / phi;

    std::vector<Vec3> v;
    v.reserve(20);

    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                v.push_back({(float)sx * s, (float)sy * s, (float)sz * s});
            }
        }
    }
    
    for (int sy = -1; sy <= 1; sy += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            v.push_back({0.0f, (float)sy * invPhi * s, (float)sz * phi * s});
        }
    }
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            v.push_back({(float)sx * invPhi * s, (float)sy * phi * s, 0.0f});
        }
    }
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sz = -1; sz <= 1; sz += 2) {
            v.push_back({(float)sx * phi * s, 0.0f, (float)sz * invPhi * s});
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

static bool uiButton(Rectangle r, const char* text) {
    Vector2 m = GetMousePosition();
    bool hot = CheckCollisionPointRec(m, r);
    Color bg = hot ? Color{45, 45, 45, 255} : Color{25, 25, 25, 255};
    DrawRectangleRec(r, bg);
    DrawRectangleLinesEx(r, 1.0f, Color{70, 70, 70, 255});

    int tw = MeasureText(text, 18);
    int tx = (int)(r.x + 0.5f * (r.width - (float)tw));
    int ty = (int)(r.y + 0.5f * (r.height - 18.0f));
    DrawText(text, tx, ty, 18, RAYWHITE);
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
    std::unordered_map<const RigidBody*, Renderer::RenderStyle> bodyStyles;

    std::vector<MaterialProps> presetMaterials = loadMaterialsFromFolder("materials");
    MaterialProps customTemplate;
    customTemplate.name = "Custom";
    customTemplate.color = Color{155, 155, 155, 255};
    customTemplate.opacity = 1.0f;
    customTemplate.outline = true;

    MaterialProps currentMaterial = presetMaterials.empty() ? customTemplate : presetMaterials[0];
    bool currentMaterialIsCustom = presetMaterials.empty();

    MaterialProps floorMaterial = presetMaterials.empty() ? customTemplate : presetMaterials[0];
    bool floorMaterialIsCustom = presetMaterials.empty();

    SpawnShape currentShape = SpawnShape::Cube;
    float currentSize = 0.35f;

    SpawnMode spawnMode = SpawnMode::Throw;


    Renderer renderer;
    if (!renderer.init(1280, 720, "Kinematica Sandbox")) return -1;
    int windowedW = 1280;
    int windowedH = 720;
    int windowedX = 0;
    int windowedY = 0;
    {
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
                const float volume = (half.x * 2.0f) * (half.y * 2.0f) * (half.z * 2.0f);
                b.mass = std::max(0.05f, std::max(1.0f, wallMaterial.density) * std::max(0.0f, volume));
                b.friction = std::clamp(wallMaterial.friction, 0.0f, 2.0f);
                b.restitution = std::clamp(wallMaterial.restitution, 0.0f, 1.0f);
                b.collider = Collider::createBox(half);
                b.isStatic = false;
                b.sleeping = true;
                b.sleepTimer = 1.0f;

                dynamicBodies.push_back(b);
                physicsWorld.addRigidBody(&dynamicBodies.back());
                {
                    const RigidBody* rb = &dynamicBodies.back();
                    Renderer::RenderStyle st;
                    st.color = applyOpacity(wallMaterial.color, wallMaterial.opacity);
                    st.outline = wallMaterial.outline;
                    bodyStyles[rb] = st;
                }
            }
        }
    }

    SetExitKey(0);

    while (!WindowShouldClose()) {
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
        accumulator += frameTime;

        physicsWorld.floorFriction = std::clamp(floorMaterial.friction, 0.0f, 2.0f);
        physicsWorld.floorRestitution = std::clamp(floorMaterial.restitution, 0.0f, 1.0f);

        if (IsKeyPressed(KEY_Q)) {
            const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            Vec3 camPos = renderer.getCameraPosition();
            Vec3 camFwd = renderer.getCameraForward();
            camFwd = camFwd.normalized();

            Vec3 up = {0.0f, 1.0f, 0.0f};
            Vec3 right = Vec3::cross(up, camFwd).normalized();
            if (right.lengthSq() < 1e-8f) right = {1.0f, 0.0f, 0.0f};

            RigidBody b;
            b.position = camPos + camFwd * 2.0f;
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

            b.isStatic = (spawnMode == SpawnMode::PlaceStatic);
            b.sleeping = (spawnMode != SpawnMode::Throw);
            b.sleepTimer = (spawnMode != SpawnMode::Throw) ? 1.0f : 0.0f;

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

                    if (spawnMode == SpawnMode::Throw) {
                        b.position = camPos + camFwd * 2.0f;
                        b.velocity = camFwd * 10.0f;
                        b.angularVelocity = {0.0f, 0.0f, 0.0f};
                    }

                    volume = width * length * std::max(0.01f, height) * 0.5f;
                    break;
                }
            }

            if (spawnMode != SpawnMode::Throw) {
                RayHit hit = raycastWorldPlacement(camPos, camFwd, physicsWorld.floorY, dynamicBodies);
                Vec3 placePoint = camPos + camFwd * 2.0f;
                Vec3 placeNormal = {0.0f, 1.0f, 0.0f};
                if (hit.hit) {
                    placePoint = hit.point;
                    placeNormal = hit.normal;
                }
                float offset = placementOffsetAlongNormal(b.collider, b.orientation, placeNormal);
                b.position = placePoint + placeNormal.normalized() * (offset + 0.002f);
                b.velocity = {0.0f, 0.0f, 0.0f};
                b.angularVelocity = {0.0f, 0.0f, 0.0f};
            }

            float density = std::max(1.0f, currentMaterial.density);
            b.mass = std::max(0.05f, density * std::max(0.0f, volume));

            dynamicBodies.push_back(b);
            physicsWorld.addRigidBody(&dynamicBodies.back());
            {
                const RigidBody* rb = &dynamicBodies.back();
                Renderer::RenderStyle st;
                st.color = applyOpacity(currentMaterial.color, currentMaterial.opacity);
                st.outline = currentMaterial.outline;
                bodyStyles[rb] = st;
            }
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
                Renderer::RenderStyle st = getStyle(&body);
                if (st.color.a < 255) transparent.push_back(&body);
                else opaque.push_back(&body);
            }

            for (const RigidBody* rb : opaque) {
                renderer.drawRigidBody(rb, getStyle(rb));
            }

            std::sort(transparent.begin(), transparent.end(), [&](const RigidBody* a, const RigidBody* b) {
                Vec3 da = a->position - camPos;
                Vec3 db = b->position - camPos;
                return da.lengthSq() > db.lengthSq();
            });
            for (const RigidBody* rb : transparent) {
                renderer.drawRigidBody(rb, getStyle(rb));
            }

            if (spawnMode != SpawnMode::Throw) {
                Vec3 camFwd = renderer.getCameraForward().normalized();
                const bool shiftDown = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
                RayHit hit = raycastWorldPlacement(camPos, camFwd, physicsWorld.floorY, dynamicBodies);
                Vec3 placePoint = camPos + camFwd * 2.0f;
                Vec3 placeNormal = {0.0f, 1.0f, 0.0f};
                if (hit.hit) {
                    placePoint = hit.point;
                    placeNormal = hit.normal;
                }

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
                        ghost.collider = Collider::createConvex(makeTetraVerts(s));
                        break;
                    case SpawnShape::Bipyramid:
                        ghost.collider = Collider::createConvex(makePentagonalBipyramidVerts(s));
                        break;
                    case SpawnShape::Dodecahedron:
                        ghost.collider = Collider::createConvex(makeDodecaVerts(s));
                        break;
                    case SpawnShape::Ramp: {
                        float width = 2.5f * s;
                        float length = 4.0f * s;
                        float height = 1.4f * s;
                        auto rampRender = makeRampMesh(width, length, height);
                        auto rampCollision = shiftDown ? rampRender : makeRampCollisionMesh(width, length, height);
                        ghost.collider = Collider::createMesh(rampCollision, rampRender);
                        Vec3 f = {camFwd.x, 0.0f, camFwd.z};
                        if (f.lengthSq() < 1e-8f) f = {0.0f, 0.0f, 1.0f};
                        f = f.normalized();
                        float yaw = atan2f(f.x, f.z);
                        ghost.orientation = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, yaw).normalized();
                        break;
                    }
                }

                float offset = placementOffsetAlongNormal(ghost.collider, ghost.orientation, placeNormal);
                ghost.position = placePoint + placeNormal.normalized() * (offset + 0.002f);

                Renderer::RenderStyle holo;
                holo.color = Color{0, 190, 255, 90};
                holo.outline = true;
                renderer.drawRigidBody(&ghost, holo);
            }
        }
        renderer.end3D();
        {
            static float uiScroll = 0.0f;
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
            if (uiButton(toggle, hudCollapsed ? "<" : ">")) {
                hudCollapsed = !hudCollapsed;
            }

            if (hudCollapsed) {
                renderer.setUiBlockRect(panel);
                renderer.endFrame();
                continue;
            }

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

            DrawText("Spawn Mode", (int)x, (int)y, 18, RAYWHITE);
            y += 24.0f;
            {
                Rectangle r1 = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r1, "Throw (Dynamic)", spawnMode == SpawnMode::Throw)) spawnMode = SpawnMode::Throw;
                y += 28.0f;

                Rectangle r2 = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r2, "Place (Dynamic)", spawnMode == SpawnMode::PlaceDynamic)) spawnMode = SpawnMode::PlaceDynamic;
                y += 28.0f;

                Rectangle r3 = {x, y, panel.width - 24.0f, 24.0f};
                if (uiRadio(r3, "Place (Static)", spawnMode == SpawnMode::PlaceStatic)) spawnMode = SpawnMode::PlaceStatic;
                y += 34.0f;
            }

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
            DrawText("Q to Spawn", (int)(panel.x + pad), (int)(footer.y + 12.0f), 18, RAYWHITE);
        }

        renderer.endFrame();
    }

    renderer.shutdown();
    return 0;
}