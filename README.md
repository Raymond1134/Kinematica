
# Kinematica Physics Engine

**Version 1.0**

A lightweight 3D physics engine and sandbox built with C++ and Raylib.

## Features

- **Rigid Body Dynamics**: Sohere, Box, Capsule, Convex Hull, and Mesh colliders.
- **Soft Body Dynamics**: Cloth simulation and Jelly-like blobs.
- **Compound Shapes**: Vehicles and complex objects.
- **Constraints**: Ball-Socket and Hinge joints.
- **Continuous Collision Detection (CCD)**: Prevents tunneling of fast-moving objects.
- **Interactive Sandbox**: Spawn, throw, and manipulate objects in real-time.
- **Visual Polish**: Lighting, holograms, instanced rendering, and clean UI.

## Controls

| Key / Action | Description |
| --- | --- |
| **WASD** | Move Camera (Free-fly) |
| **Right Click + Drag** | Look Around / Orbit |
| **Space** | Move Camera Up |
| **Left Shift** | Move Camera Down |
| **Mouse Wheel** | Zoom In / Out |
| **Q** | Spawn Selected Shape |
| **K** | Kill All (Clear Scene) |
| **R** | Reset Scene |
| **P** | Pause / Resume Simulation |
| **.** | Single Step (when paused) |
| **[ / ]** | Decrease / Increase Time Scale |
| **Alt + Enter** | Toggle Fullscreen |
| **Left Click** | Interact with UI (Right Panel) |

## Build Instructions (Windows)

### Prerequisites
- **Visual Studio 2022** (or C++ Build Tools)
- **CMake** (3.20+)
- **Raylib** (Managed via FetchContent or vcpkg)

### Building with VS Code (Recommended)
1. Open the folder in VS Code.
2. Press `Ctrl+Shift+B` to build.
3. Run the **"Run (Release)"** task.

### Building from Command Line
```powershell
# Configure
cmake -S . -B build

# Build
cmake --build build --config Release

# Run
.\build\Release\app.exe
```

## Documentation

See the `docs/` folder for detailed documentation:
- [API Reference](docs/API.md)
- [Architecture Overview](docs/ARCHITECTURE.md)
- [Examples](docs/EXAMPLES.md)

## Project Structure

- `physics/` — Core physics engine (`PhysicsWorld`, `RigidBody`, `Solver`).
- `math/` — Vector and Quaternion math library.
- `render/` — Rendering system using Raylib.
- `materials/` — JSON-based material definitions.
- `main.cpp` — Sandbox application entry point.
