#include "SoftBody.h"
#include "RigidBody.h"
#include "../render/Renderer.h"
#include <raymath.h>
#include <rlgl.h>
#include <cstring>
#include <cmath>

void SoftBody::init(int r, int c, Color col) {
    rows = r;
    cols = c;
    layers = 1;
    isBlob = false;
    color = col;

    mesh = { 0 };
    mesh.vertexCount = rows * cols;
    mesh.triangleCount = (rows - 1) * (cols - 1) * 2;

    mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
    mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

    int k = 0;
    for (int i = 0; i < rows - 1; i++) {
        for (int j = 0; j < cols - 1; j++) {
            int nextRow = i + 1;
            int nextCol = j + 1;

            int idx0 = i * cols + j;
            int idx1 = nextRow * cols + j;
            int idx2 = nextRow * cols + nextCol;
            int idx3 = i * cols + nextCol;

            mesh.indices[k++] = idx0;
            mesh.indices[k++] = idx1;
            mesh.indices[k++] = idx2;

            mesh.indices[k++] = idx0;
            mesh.indices[k++] = idx2;
            mesh.indices[k++] = idx3;
        }
    }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            mesh.texcoords[(i * cols + j) * 2] = (float)j / (cols - 1);
            mesh.texcoords[(i * cols + j) * 2 + 1] = (float)i / (rows - 1);
            
            mesh.vertices[(i * cols + j) * 3] = 0;
            mesh.vertices[(i * cols + j) * 3 + 1] = 0;
            mesh.vertices[(i * cols + j) * 3 + 2] = 0;
            
            mesh.normals[(i * cols + j) * 3] = 0;
            mesh.normals[(i * cols + j) * 3 + 1] = 1;
            mesh.normals[(i * cols + j) * 3 + 2] = 0;
        }
    }

    UploadMesh(&mesh, true);
    model = LoadModelFromMesh(mesh);
}

void SoftBody::initBlob(int r, int c, int l, Color col) {
    rows = r;
    cols = c;
    layers = l;
    isBlob = true;
    color = col;
    
    mesh = { 0 };
    mesh.vertexCount = rows * cols * layers;
    
    int hullQuads = 2 * ((rows-1)*(cols-1) + (rows-1)*(layers-1) + (cols-1)*(layers-1));
    mesh.triangleCount = hullQuads * 4;

    mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
    mesh.indices = (unsigned short*)MemAlloc(mesh.triangleCount * 3 * sizeof(unsigned short));

    int k = 0;
    auto addQuad = [&](int i0, int i1, int i2, int i3) {
        mesh.indices[k++] = i0; mesh.indices[k++] = i1; mesh.indices[k++] = i2;
        mesh.indices[k++] = i0; mesh.indices[k++] = i2; mesh.indices[k++] = i3;
        mesh.indices[k++] = i0; mesh.indices[k++] = i2; mesh.indices[k++] = i1;
        mesh.indices[k++] = i0; mesh.indices[k++] = i3; mesh.indices[k++] = i2;
    };
    
    auto getIdx = [&](int x, int y, int z) { return z * rows * cols + y * cols + x; };

    for (int y = 0; y < rows - 1; y++) {
        for (int x = 0; x < cols - 1; x++) {
            addQuad(getIdx(x, y, layers-1), getIdx(x+1, y, layers-1), getIdx(x+1, y+1, layers-1), getIdx(x, y+1, layers-1));
        }
    }
    for (int y = 0; y < rows - 1; y++) {
        for (int x = 0; x < cols - 1; x++) {
            addQuad(getIdx(x, y, 0), getIdx(x, y+1, 0), getIdx(x+1, y+1, 0), getIdx(x+1, y, 0));
        }
    }
    for (int z = 0; z < layers - 1; z++) {
        for (int x = 0; x < cols - 1; x++) {
            addQuad(getIdx(x, rows-1, z), getIdx(x+1, rows-1, z), getIdx(x+1, rows-1, z+1), getIdx(x, rows-1, z+1));
        }
    }
    for (int z = 0; z < layers - 1; z++) {
        for (int x = 0; x < cols - 1; x++) {
            addQuad(getIdx(x, 0, z), getIdx(x, 0, z+1), getIdx(x+1, 0, z+1), getIdx(x+1, 0, z));
        }
    }
    for (int z = 0; z < layers - 1; z++) {
        for (int y = 0; y < rows - 1; y++) {
            addQuad(getIdx(cols-1, y, z), getIdx(cols-1, y+1, z), getIdx(cols-1, y+1, z+1), getIdx(cols-1, y, z+1));
        }
    }
    for (int z = 0; z < layers - 1; z++) {
        for (int y = 0; y < rows - 1; y++) {
            addQuad(getIdx(0, y, z), getIdx(0, y, z+1), getIdx(0, y+1, z+1), getIdx(0, y+1, z));
        }
    }

    for (int i = 0; i < mesh.vertexCount; i++) {
        mesh.vertices[i*3] = 0; mesh.vertices[i*3+1] = 0; mesh.vertices[i*3+2] = 0;
        mesh.normals[i*3] = 0; mesh.normals[i*3+1] = 1; mesh.normals[i*3+2] = 0;
        mesh.texcoords[i*2] = 0; mesh.texcoords[i*2+1] = 0;
    }

    UploadMesh(&mesh, true);
    model = LoadModelFromMesh(mesh);
}

void SoftBody::update() {
    if (particles.empty()) return;

    Vec3 minV = {1e30f, 1e30f, 1e30f};
    Vec3 maxV = {-1e30f, -1e30f, -1e30f};

    for (int i = 0; i < mesh.vertexCount; i++) {
        if (i < (int)particles.size() && particles[i]) {
            Vec3 p = particles[i]->position;
            mesh.vertices[i * 3] = p.x;
            mesh.vertices[i * 3 + 1] = p.y;
            mesh.vertices[i * 3 + 2] = p.z;

            if (p.x < minV.x) minV.x = p.x;
            if (p.y < minV.y) minV.y = p.y;
            if (p.z < minV.z) minV.z = p.z;
            if (p.x > maxV.x) maxV.x = p.x;
            if (p.y > maxV.y) maxV.y = p.y;
            if (p.z > maxV.z) maxV.z = p.z;
        }
    }

    boundsCenter = (minV + maxV) * 0.5f;
    boundsRadius = (maxV - minV).length() * 0.5f;

    if (mesh.normals == nullptr) return;
    memset(mesh.normals, 0, mesh.vertexCount * 3 * sizeof(float));

    if (mesh.indices == nullptr) return;
    for (int i = 0; i < mesh.triangleCount; i++) {
        int idx1 = mesh.indices[i * 3];
        int idx2 = mesh.indices[i * 3 + 1];
        int idx3 = mesh.indices[i * 3 + 2];

        Vec3 v1 = { mesh.vertices[idx1 * 3], mesh.vertices[idx1 * 3 + 1], mesh.vertices[idx1 * 3 + 2] };
        Vec3 v2 = { mesh.vertices[idx2 * 3], mesh.vertices[idx2 * 3 + 1], mesh.vertices[idx2 * 3 + 2] };
        Vec3 v3 = { mesh.vertices[idx3 * 3], mesh.vertices[idx3 * 3 + 1], mesh.vertices[idx3 * 3 + 2] };

        Vec3 normal = Vec3::cross(v2 - v1, v3 - v1).normalized();

        mesh.normals[idx1 * 3] += normal.x; mesh.normals[idx1 * 3 + 1] += normal.y; mesh.normals[idx1 * 3 + 2] += normal.z;
        mesh.normals[idx2 * 3] += normal.x; mesh.normals[idx2 * 3 + 1] += normal.y; mesh.normals[idx2 * 3 + 2] += normal.z;
        mesh.normals[idx3 * 3] += normal.x; mesh.normals[idx3 * 3 + 1] += normal.y; mesh.normals[idx3 * 3 + 2] += normal.z;
    }

    for (int i = 0; i < mesh.vertexCount; i++) {
        float nx = mesh.normals[i * 3];
        float ny = mesh.normals[i * 3 + 1];
        float nz = mesh.normals[i * 3 + 2];
        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len > 1e-6f) {
            mesh.normals[i * 3] /= len;
            mesh.normals[i * 3 + 1] /= len;
            mesh.normals[i * 3 + 2] /= len;
        }
    }

    UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);
    UpdateMeshBuffer(mesh, 2, mesh.normals, mesh.vertexCount * 3 * sizeof(float), 0);
}

void SoftBody::draw(const Renderer& renderer) {
    if (!renderer.isVisible(boundsCenter, boundsRadius)) return;
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
    if (!isBlob) {
        rlDisableBackfaceCulling();
        DrawModel(model, { 0,0,0 }, 1.0f, WHITE);
        rlEnableBackfaceCulling();
    } else {
        DrawModel(model, { 0,0,0 }, 1.0f, WHITE);
    }

    if (isBlob) return;
    if (rows < 2 || cols < 2) return;
    if ((int)particles.size() < rows * cols) return;

    const Vec3 cam = renderer.getCameraPosition();
    auto p3 = [&](int idx) -> Vector3 {
        Vec3 p = particles[idx] ? particles[idx]->position : Vec3{0.0f, 0.0f, 0.0f};
        Vec3 toCam = cam - p;
        float d2 = toCam.lengthSq();
        if (d2 > 1e-10f) {
            toCam = toCam * (1.0f / sqrtf(d2));
            p = p + toCam * 0.0020f;
        }
        return { p.x, p.y, p.z };
    };

    constexpr Color kEdge = { 0, 0, 0, 255 };

    for (int j = 0; j < cols - 1; ++j) {
        DrawLine3D(p3(0 * cols + j), p3(0 * cols + (j + 1)), kEdge);
    }

    for (int j = 0; j < cols - 1; ++j) {
        DrawLine3D(p3((rows - 1) * cols + j), p3((rows - 1) * cols + (j + 1)), kEdge);
    }

    for (int i = 0; i < rows - 1; ++i) {
        DrawLine3D(p3(i * cols + 0), p3((i + 1) * cols + 0), kEdge);
    }

    for (int i = 0; i < rows - 1; ++i) {
        DrawLine3D(p3(i * cols + (cols - 1)), p3((i + 1) * cols + (cols - 1)), kEdge);
    }
}
