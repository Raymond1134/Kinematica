# Kinematica Examples

This document provides practical examples of how to use the Kinematica Physics Engine API.

## 1. Basic Setup

Initialize the world and the factory.

```cpp
#include "physics/PhysicsWorld.h"
#include "physics/PhysicsFactory.h"

PhysicsWorld world;
PhysicsFactory factory(world, world.bodies);

// Define a material
MaterialProps wood;
wood.name = "Wood";
wood.density = 0.7f;
wood.friction = 0.6f;
wood.restitution = 0.2f;
wood.color = {139, 69, 19, 255};
```

## 2. Creating Primitives

```cpp
// Create a static floor
MaterialProps floorMat;
floorMat.friction = 0.8f;
factory.CreateBox({0, -1, 0}, {50, 1, 50}, floorMat, 0.0f, true);

// Create a dynamic box
factory.CreateBox({0, 5, 0}, {1, 1, 1}, wood, 10.0f);

// Create a sphere
factory.CreateSphere({2, 5, 0}, 0.5f, wood, 5.0f);
```

## 3. Using the Compound Builder

The `CompoundBuilder` makes it easy to create complex shapes by combining primitives.

```cpp
#include "physics/CompoundBuilder.h"

// Create a "Dumbbell" shape
CompoundBuilder dumbbell;
dumbbell.AddSphere({-1.5f, 0, 0}, 0.5f)   // Left weight
        .AddSphere({ 1.5f, 0, 0}, 0.5f)   // Right weight
        .AddCapsule({0, 0, 0}, 0.2f, 3.0f, Quat::fromAxisAngle({0,0,1}, 1.57f)); // Handle

// Spawn it
factory.CreateCompound({0, 10, 0}, dumbbell, wood, 15.0f);
```

## 4. Creating a Chain

Create a rope or chain using the prefab helper.

```cpp
// Create a chain of 10 links hanging down
Vec3 start = {0, 15, 0};
Vec3 dir = {0, -1, 0};
factory.CreateChain(start, dir, 10, 0.2f, 0.45f, wood, true);
```

## 5. Soft Bodies

```cpp
// Create a hanging cloth
MaterialProps clothMat;
clothMat.color = {200, 50, 50, 255};
factory.CreateCloth({0, 10, 0}, 10, 10, 0.3f, clothMat, true);

// Create a jelly blob
factory.CreateBlob({5, 5, 5}, 4, 0.4f, 0.2f, clothMat);
```

## 6. Simulation Loop

```cpp
float dt = 1.0f / 60.0f;
while (appRunning) {
    // Step physics
    world.step(dt);

    // Render bodies...
    for (RigidBody* body : world.bodies) {
        DrawBody(body);
    }
}
```
