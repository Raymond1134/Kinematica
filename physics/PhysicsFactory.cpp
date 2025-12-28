#include "PhysicsFactory.h"
#include <algorithm>
#include <cmath>

PhysicsFactory::PhysicsFactory(PhysicsWorld& world, std::list<RigidBody>& bodies)
    : world(world), bodies(bodies) {}

RigidBody* PhysicsFactory::CreateBodyBase(const Vec3& pos, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody body;
    body.position = pos;
    body.velocity = vel;
    body.orientation = Quat::identity();
    body.angularVelocity = {0.0f, 0.0f, 0.0f};
    body.mass = mass;
    body.friction = mat.friction;
    body.restitution = mat.restitution;
    body.isStatic = isStatic;
    if (isStatic) body.mass = 0.0f;
    
    body.useExplicitInertia = false;

    bodies.push_back(body);
    RigidBody* ptr = &bodies.back();
    world.addRigidBody(ptr);
    return ptr;
}

RigidBody* PhysicsFactory::CreateBox(const Vec3& pos, const Vec3& size, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody* body = CreateBodyBase(pos, mat, mass, isStatic, vel);
    body->collider = Collider::createBox(size);
    return body;
}

RigidBody* PhysicsFactory::CreateSphere(const Vec3& pos, float radius, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody* body = CreateBodyBase(pos, mat, mass, isStatic, vel);
    body->collider = Collider::createSphere(radius);
    return body;
}

RigidBody* PhysicsFactory::CreateCapsule(const Vec3& pos, float radius, float height, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody* body = CreateBodyBase(pos, mat, mass, isStatic, vel);
    body->collider = Collider::createCapsule(radius, height);
    return body;
}

RigidBody* PhysicsFactory::CreateConvex(const Vec3& pos, const std::vector<Vec3>& verts, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody* body = CreateBodyBase(pos, mat, mass, isStatic, vel);
    body->collider = Collider::createConvex(verts);
    return body;
}

RigidBody* PhysicsFactory::CreateMesh(const Vec3& pos, std::shared_ptr<TriangleMesh> mesh, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody* body = CreateBodyBase(pos, mat, mass, isStatic, vel);
    body->collider = Collider::createMesh(mesh, mesh);
    return body;
}

RigidBody* PhysicsFactory::CreateCompound(const Vec3& pos, const std::vector<CompoundChild>& children, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    RigidBody* body = CreateBodyBase(pos, mat, mass, isStatic, vel);
    body->collider = Collider::createCompound(children);
    return body;
}

RigidBody* PhysicsFactory::CreateCompound(const Vec3& pos, const CompoundBuilder& builder, const MaterialProps& mat, float mass, bool isStatic, const Vec3& vel) {
    return CreateCompound(pos, builder.Build(), mat, mass, isStatic, vel);
}

static float effectiveMass(float mA, float mB) {
    const bool aDyn = (mA > 0.0f);
    const bool bDyn = (mB > 0.0f);
    if (aDyn && bDyn) return (mA * mB) / (mA + mB);
    if (aDyn) return mA;
    if (bDyn) return mB;
    return 0.0f;
}

static void computeSpringParams(RigidBody* a, RigidBody* b, float restLen, float omega, float dampingRatio, float dt, float spacing, float& outK, float& outC) {
    const float mEff = effectiveMass(a ? a->mass : 0.0f, b ? b->mass : 0.0f);
    if (mEff <= 0.0f || !std::isfinite(mEff)) { outK = 0.0f; outC = 0.0f; return; }

    const float lenScale = std::clamp(spacing / std::max(1e-6f, restLen), 0.25f, 1.0f);

    float k = (omega * omega) * mEff * lenScale;

    const float omegaMax = 2.0f / std::max(1e-6f, dt);
    const float kMax = 0.95f * (omegaMax * omegaMax) * mEff;
    if (k > kMax) k = kMax;

    float c = 2.0f * dampingRatio * std::sqrt(std::max(0.0f, k * mEff));
    outK = k;
    outC = c;
}

SoftBody PhysicsFactory::CreateCloth(const Vec3& pos, int rows, int cols, float spacing, const MaterialProps& mat, bool isStatic, const Vec3& vel) {
    SoftBody cloth;
    cloth.init(rows, cols, mat.color);
    cloth.particles.reserve(rows * cols);

    static int nextGroupId = 1;
    int clothGroupId = nextGroupId++;
    
    float radius = spacing * 0.6f;
    float vol = (4.0f / 3.0f) * 3.14159f * radius * radius * radius;
    float mass = vol * mat.density;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Vec3 pPos = pos + Vec3{(float)(j - cols/2) * spacing, 0.0f, (float)(i - rows/2) * spacing};
            bool pStatic = isStatic && (i == 0);
            
            RigidBody* p = CreateBodyBase(pPos, mat, mass, pStatic, vel);
            p->collider = Collider::createSphere(radius);
            p->visible = false;
            p->groupId = clothGroupId;
            p->useExplicitInertia = true;
            p->explicitInvInertia = {0.0f, 0.0f, 0.0f};
            
            cloth.particles.push_back(p);
        }
    }

    const float omegaStructural = 40.0f;
    const float omegaBend = 30.0f;
    const float dampingRatio = 1.0f;
    float dt = world.currentDt;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            RigidBody* a = cloth.particles[i * cols + j];

            auto addSpring = [&](int r, int c, float rest, float omega) {
                if (r >= rows || c >= cols || r < 0 || c < 0) return;
                RigidBody* b = cloth.particles[r * cols + c];
                float k, damping;
                computeSpringParams(a, b, rest, omega, dampingRatio, dt, spacing, k, damping);
                world.addSpring(a, b, rest, k, damping);
            };

            if (j + 1 < cols) addSpring(i, j + 1, spacing, omegaStructural);
            if (i + 1 < rows) addSpring(i + 1, j, spacing, omegaStructural);
            
            float diag = spacing * 1.4142f;
            if (i + 1 < rows && j + 1 < cols) addSpring(i + 1, j + 1, diag, omegaStructural);
            if (i + 1 < rows && j - 1 >= 0) addSpring(i + 1, j - 1, diag, omegaStructural);
            
            float rest2 = spacing * 2.0f;
            if (j + 2 < cols) addSpring(i, j + 2, rest2, omegaBend);
            if (i + 2 < rows) addSpring(i + 2, j, rest2, omegaBend);
        }
    }
    return cloth;
}

SoftBody PhysicsFactory::CreateBlob(const Vec3& pos, int dim, float spacing, float radius, const MaterialProps& mat, bool isStatic, const Vec3& vel) {
    SoftBody blob;
    blob.initBlob(dim, dim, dim, mat.color);
    blob.particles.reserve(dim * dim * dim);

    static int nextGroupId = 1;
    int blobGroupId = nextGroupId++;

    float vol = (4.0f / 3.0f) * 3.14159f * radius * radius * radius;
    float mass = vol * mat.density;
    float stiffness = 6.0f * mat.density;
    float damping = 0.1f * mat.density;

    for (int z = 0; z < dim; ++z) {
        for (int y = 0; y < dim; ++y) {
            for (int x = 0; x < dim; ++x) {
                Vec3 pPos = pos + Vec3{(float)(x - 1) * spacing, (float)(y - 1) * spacing, (float)(z - 1) * spacing};
                
                RigidBody* p = CreateBodyBase(pPos, mat, mass, isStatic, vel);
                p->collider = Collider::createSphere(radius);
                p->visible = false;
                p->groupId = blobGroupId;
                p->useExplicitInertia = true;
                p->explicitInvInertia = {0.0f, 0.0f, 0.0f};

                blob.particles.push_back(p);
            }
        }
    }

    auto idx = [&](int x, int y, int z) { return z * dim * dim + y * dim + x; };

    for (int z = 0; z < dim; ++z) {
        for (int y = 0; y < dim; ++y) {
            for (int x = 0; x < dim; ++x) {
                RigidBody* a = blob.particles[idx(x, y, z)];

                auto addSpring = [&](int nx, int ny, int nz, float rest) {
                    if (nx >= dim || ny >= dim || nz >= dim || nx < 0 || ny < 0 || nz < 0) return;
                    RigidBody* b = blob.particles[idx(nx, ny, nz)];
                    world.addSpring(a, b, rest, stiffness, damping);
                };

                addSpring(x + 1, y, z, spacing);
                addSpring(x, y + 1, z, spacing);
                addSpring(x, y, z + 1, spacing);

                float diag2 = spacing * 1.4142f;
                addSpring(x + 1, y + 1, z, diag2);
                addSpring(x + 1, y - 1, z, diag2);
                addSpring(x, y + 1, z + 1, diag2);
                addSpring(x, y + 1, z - 1, diag2);
                addSpring(x + 1, y, z + 1, diag2);
                addSpring(x + 1, y, z - 1, diag2);

                float diag3 = spacing * 1.732f;
                addSpring(x + 1, y + 1, z + 1, diag3);
                addSpring(x + 1, y + 1, z - 1, diag3);
                addSpring(x + 1, y - 1, z + 1, diag3);
                addSpring(x + 1, y - 1, z - 1, diag3);
            }
        }
    }
    return blob;
}

RigidBody* PhysicsFactory::CreateCar(const Vec3& pos, float size, const MaterialProps& mat, bool isStatic, const Vec3& vel) {
    Vec3 chassisDim = {1.0f * size, 0.25f * size, 2.0f * size};
    float wheelRadius = 0.3f * size;
    Vec3 wheelOffsets[4] = {
        {-0.6f * size, -0.1f * size, -0.7f * size},
        { 0.6f * size, -0.1f * size, -0.7f * size},
        {-0.6f * size, -0.1f * size,  0.7f * size},
        { 0.6f * size, -0.1f * size,  0.7f * size}
    };

    std::vector<CompoundChild> carChildren;
    carChildren.reserve(5);
    {
        CompoundChild body;
        body.collider = Collider::createBox(chassisDim * 0.5f);
        body.localPosition = {0.0f, 0.0f, 0.0f};
        body.localOrientation = Quat::identity();
        carChildren.push_back(body);
    }
    for (int i = 0; i < 4; ++i) {
        CompoundChild w;
        w.collider = Collider::createSphere(wheelRadius);
        w.localPosition = wheelOffsets[i];
        w.localOrientation = Quat::identity();
        carChildren.push_back(w);
    }
    
    float chassisVol = chassisDim.x * chassisDim.y * chassisDim.z;
    float wheelVol = (4.0f / 3.0f) * 3.14159f * wheelRadius * wheelRadius * wheelRadius;
    float totalVol = chassisVol + 4.0f * wheelVol;
    float mass = totalVol * mat.density;

    return CreateCompound(pos, carChildren, mat, mass, isStatic, vel);
}

std::vector<RigidBody*> PhysicsFactory::CreateChain(const Vec3& startPos, const Vec3& dir, int length, float radius, float spacing, const MaterialProps& mat, bool startStatic, const Vec3& vel) {
    std::vector<RigidBody*> chainBodies;
    chainBodies.reserve(length);
    RigidBody* prev = nullptr;
    
    float density = std::max(1.0f, mat.density);
    float volume = (4.0f / 3.0f) * 3.14159f * radius * radius * radius;
    float mass = std::max(0.05f, density * volume);

    for (int i = 0; i < length; ++i) {
        bool isStatic = (i == 0 && startStatic);
        Vec3 pos = startPos + dir * ((float)i * spacing);
        
        RigidBody* curr = CreateSphere(pos, radius, mat, mass, isStatic, vel);
        curr->sleeping = false;
        curr->sleepTimer = 0.0f;
        chainBodies.push_back(curr);

        if (prev) {
            Vec3 anchor = (prev->position + curr->position) * 0.5f;
            world.addBallSocketJoint(prev, curr, anchor);
        }
        prev = curr;
    }
    return chainBodies;
}
