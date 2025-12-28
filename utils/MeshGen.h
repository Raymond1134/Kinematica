#pragma once
#include <memory>
#include <vector>
#include "../math/Vec3.h"
#include "../physics/collision/TriangleMesh.h"

namespace MeshGen {
    std::shared_ptr<TriangleMesh> makeTorusMesh(float majorRadius, float minorRadius, int majorSegments, int minorSegments);
    std::shared_ptr<TriangleMesh> makeRampMesh(float width, float length, float height);
    std::shared_ptr<TriangleMesh> makeRampCollisionMesh(float width, float length, float height);
    std::vector<Vec3> makeTetraVerts(float s);
    std::vector<Vec3> makePentagonalBipyramidVerts(float s);
    std::vector<Vec3> makeDodecaVerts(float s);
}
