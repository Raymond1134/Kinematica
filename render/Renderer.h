#pragma once

#include "../math/Vec3.h"
#include "../physics/shapes/PolyhedronShape.h"
#include <vector>
#include <cstdint>
#include <raylib.h>
#include <unordered_map>
#include <memory>

struct RigidBody;
struct TriangleMesh;

class Renderer {
public:
    struct RenderStyle {
        Color color = BLUE;
        bool outline = false;
        std::shared_ptr<const TriangleMesh> meshOverride;
    };

    bool init(int width, int height, const char* title);
    void beginFrame();
    void drawRigidBody(const RigidBody* body, const RenderStyle& style, float size = 0.5f);
    void drawConvex(const PolyhedronShape& poly, Color color, bool outline) const;
    void end3D();
    void setUiBlockRect(Rectangle r);
    void endFrame();
    void shutdown();
    
    Vec3 getCameraPosition() const;
    Vec3 getCameraForward() const;

private:
    bool in3D = false;
    Vec3 camPos = {8.0f, 6.0f, 8.0f};
    float camYaw = 225.0f;
    float camPitch = -20.0f;
    float moveSpeed = 8.0f; // meters/sec

    bool hasUiBlockRect = false;
    Rectangle uiBlockRect = {0.0f, 0.0f, 0.0f, 0.0f};

    struct Edge {
        uint32_t a = 0;
        uint32_t b = 0;
    };
    struct MeshEdgeCache {
        const TriangleMesh* mesh = nullptr;
        std::vector<Edge> boundaryEdges;
    };
    mutable std::vector<MeshEdgeCache> meshEdgeCaches;
    const std::vector<Edge>& getMeshBoundaryEdges(const TriangleMesh& mesh) const;

    Mesh cubeMesh = { 0 };
    Mesh sphereMesh = { 0 };
    bool meshesInitialized = false;

    struct InstanceData {
        Matrix transform;
        Color color;
    };
    std::unordered_map<unsigned int, std::vector<Matrix>> cubeInstances;
    std::unordered_map<unsigned int, std::vector<Matrix>> sphereInstances;
};
