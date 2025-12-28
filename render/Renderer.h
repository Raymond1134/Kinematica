#pragma once

#include "../math/Vec3.h"
#include "../physics/shapes/PolyhedronShape.h"
#include "Frustum.h"
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
        bool grid = false;
        std::shared_ptr<const TriangleMesh> meshOverride;
    };

    bool init(int width, int height, const char* title);
    void beginFrame();
    void flush();
    void drawRigidBody(const RigidBody* body, const RenderStyle& style, float size = 0.5f);
    void drawConvex(const PolyhedronShape& poly, Color color, bool outline) const;
    void end3D();
    void setUiBlockRect(Rectangle r);
    void endFrame();
    void shutdown();
    
    bool isVisible(const Vec3& center, float radius) const;

    Vec3 getCameraPosition() const;
    Vec3 getCameraForward() const;

private:
    Frustum frustum;
    bool in3D = false;
    Camera3D mainCam = { 0 };
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

    Shader lightingShader = { 0 };
    int locViewPos = -1;
    int locLightDir = -1;
    int locLightColor = -1;
    int locAmbientColor = -1;
    int locUseGrid = -1;

    struct InstanceKey {
        uint32_t color;
        bool grid;
        
        bool operator==(const InstanceKey& other) const {
            return color == other.color && grid == other.grid;
        }
    };
    
    struct InstanceKeyHash {
        std::size_t operator()(const InstanceKey& k) const {
            return std::hash<uint32_t>()(k.color) ^ (std::hash<bool>()(k.grid) << 1);
        }
    };

    std::unordered_map<InstanceKey, std::vector<Matrix>, InstanceKeyHash> cubeInstances;
    std::unordered_map<InstanceKey, std::vector<Matrix>, InstanceKeyHash> sphereInstances;
};
