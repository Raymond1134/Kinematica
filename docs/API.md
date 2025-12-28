# Kinematica Physics Engine API

## Overview

Kinematica is a lightweight 3D physics engine written in C++. It provides a simple API for creating and simulating rigid bodies, soft bodies, and constraints.

## Units

- **Position / Length**: Meters (m)
- **Mass**: Kilograms (kg)
- **Time**: Seconds (s)
- **Velocity**: Meters per Second (m/s)
- **Force**: Newtons (N)

## PhysicsFactory

The `PhysicsFactory` class is the primary interface for creating physics objects. It handles the creation of rigid bodies, colliders, and their registration with the `PhysicsWorld`.

### Initialization

```cpp
PhysicsFactory factory(physicsWorld, bodiesList);
```

### Material Properties

The `MaterialProps` struct defines the physical and visual properties of an object.

```cpp
struct MaterialProps {
    std::string name;
    float density = 1.0f;       // kg/m^3
    float friction = 0.5f;      // 0.0 to 1.0+
    float restitution = 0.0f;   // 0.0 (no bounce) to 1.0 (full bounce)
    Color color;                // Raylib Color
    float opacity = 1.0f;       // 0.0 to 1.0
    bool outline = false;       // Draw wireframe outline
};
```

### Creation Methods

#### Primitives

```cpp
// Create a Box
RigidBody* CreateBox(const Vec3& pos, const Vec3& halfExtents, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

// Create a Sphere
RigidBody* CreateSphere(const Vec3& pos, float radius, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

// Create a Capsule
RigidBody* CreateCapsule(const Vec3& pos, float radius, float height, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});
```

#### Convex & Mesh Shapes

```cpp
// Create a Convex Hull from vertices
RigidBody* CreateConvex(const Vec3& pos, const std::vector<Vec3>& verts, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

// Create a Triangle Mesh (Static or Dynamic)
RigidBody* CreateMesh(const Vec3& pos, std::shared_ptr<TriangleMesh> mesh, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});
```

#### Compound Shapes

```cpp
#include "physics/CompoundBuilder.h"

// Create a Compound Shape from multiple children
RigidBody* CreateCompound(const Vec3& pos, const std::vector<CompoundChild>& children, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});

// Create a Compound Shape using the Builder (Recommended)
RigidBody* CreateCompound(const Vec3& pos, const CompoundBuilder& builder, const MaterialProps& mat, float mass, bool isStatic = false, const Vec3& vel = {0,0,0});
```

#### Prefabs

```cpp
// Create a Car (Chassis + 4 Wheels)
RigidBody* CreateCar(const Vec3& pos, float size, const MaterialProps& mat, bool isStatic = false, const Vec3& vel = {0,0,0});

// Create a Chain of Spheres connected by Ball-Socket Joints
std::vector<RigidBody*> CreateChain(const Vec3& startPos, const Vec3& dir, int length, float radius, float spacing, const MaterialProps& mat, bool startStatic, const Vec3& vel = {0,0,0});
```

#### Soft Bodies

```cpp
// Create a Soft Body Blob (Jelly-like)
SoftBody CreateBlob(const Vec3& pos, int dim, float spacing, float radius, const MaterialProps& mat, bool isStatic = false, const Vec3& vel = {0,0,0});

// Create a Cloth
SoftBody CreateCloth(const Vec3& pos, int rows, int cols, float spacing, const MaterialProps& mat, bool isStatic = false, const Vec3& vel = {0,0,0});
```

## PhysicsWorld

The `PhysicsWorld` class manages the simulation loop and global settings.

### Key Settings

- `gravity`: Global gravity vector (default: `{0, -9.81, 0}`).
- `enableCCD`: Enable Continuous Collision Detection (default: `true`).
- `ccdMaxSubsteps`: Max substeps for CCD (default: `24`).

### Simulation

```cpp
// Step the simulation by deltaTime seconds
world.step(deltaTime);
```
