#include "MeshGen.h"
#include <cmath>
#include <algorithm>

namespace MeshGen {

    std::shared_ptr<TriangleMesh> makeTorusMesh(float majorRadius, float minorRadius, int majorSegments, int minorSegments) {
        std::vector<Vec3> vertices;
        std::vector<uint32_t> indices;

        for (int i = 0; i <= majorSegments; ++i) {
            float u = (float)i / majorSegments * 2.0f * 3.14159f;
            float cosU = cosf(u);
            float sinU = sinf(u);

            for (int j = 0; j <= minorSegments; ++j) {
                float v = (float)j / minorSegments * 2.0f * 3.14159f;
                float cosV = cosf(v);
                float sinV = sinf(v);

                float x = (majorRadius + minorRadius * cosV) * cosU;
                float y = minorRadius * sinV;
                float z = (majorRadius + minorRadius * cosV) * sinU;

                vertices.push_back({x, y, z});
            }
        }

        for (int i = 0; i < majorSegments; ++i) {
            for (int j = 0; j < minorSegments; ++j) {
                int nextI = (i + 1);
                
                int a = i * (minorSegments + 1) + j;
                int b = nextI * (minorSegments + 1) + j;
                int c = nextI * (minorSegments + 1) + (j + 1);
                int d = i * (minorSegments + 1) + (j + 1);

                indices.push_back(a);
                indices.push_back(d);
                indices.push_back(c);

                indices.push_back(a);
                indices.push_back(c);
                indices.push_back(b);
            }
        }

        auto mesh = std::make_shared<TriangleMesh>();
        mesh->build(vertices, indices);
        return mesh;
    }

    std::shared_ptr<TriangleMesh> makeRampMesh(float width, float length, float height) {
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

    std::shared_ptr<TriangleMesh> makeRampCollisionMesh(float width, float length, float height) {
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

    std::vector<Vec3> makeTetraVerts(float s) {
        return {
            {s, s, s},
            {-s, -s, s},
            {-s, s, -s},
            {s, -s, -s},
        };
    }

    std::vector<Vec3> makePentagonalBipyramidVerts(float s) {
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

    std::vector<Vec3> makeDodecaVerts(float s) {
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

        return v;
    }
}
