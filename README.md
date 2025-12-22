
# Kinematica

A user-driven 3D rigid-body physics sandbox and engine.

## Build Options (Windows)

### Option 1: VS Code Tasks (Easiest)

If you're using VS Code, use the built-in tasks:
1. **Terminal → Run Task → "Configure (vcpkg)"** (if you have vcpkg) or **"Configure (FetchContent)"**
2. **Terminal → Run Build Task** (or press `Ctrl+Shift+B`)
3. **Terminal → Run Task → "Run (Release)"**

### Option 2: Command Line with vcpkg

1. Install vcpkg in your user directory:
   ```powershell
   cd $env:USERPROFILE
   git clone https://github.com/microsoft/vcpkg.git
   .\vcpkg\bootstrap-vcpkg.bat
   ```
2. Install raylib:
   ```powershell
   .\vcpkg\vcpkg.exe install raylib:x64-windows
   ```
3. Configure and build (vcpkg will download CMake if needed):
   ```powershell
   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="$env:USERPROFILE\vcpkg\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows
   cmake --build build --config Release
   .\build\Release\app.exe
   ```

### Option 3: Auto-Fetch raylib (no vcpkg)

CMake will download and build raylib automatically at configure time:
```powershell
cmake -S . -B build
cmake --build build --config Release
.\build\Release\app.exe
```

**Note:** If `cmake` is not in your PATH, vcpkg downloads it automatically. The VS Code tasks use the vcpkg-downloaded cmake.

## Project Layout

- `math/` — Vector math (`Vec3`)
- `physics/` — Core physics (`RigidBody`, `PhysicsWorld`)
- `render/` — 3D viewer using raylib (`Renderer`)
- `main.cpp` — Application entry with fixed-timestep physics loop
- `materials/` — Material presets (JSON)


## Controls

- **Right-click + drag** — Orbit camera
- **Mouse wheel** — Zoom in/out
- **WASD** — Move camera (free-fly)
- **Q** — Spawn the currently selected shape
- **Right-side panel** — Select shape, size, material, and floor material
- **Alt+Enter** — Toggle fullscreen
- **ESC or close window** — Exit


## Features

- User-driven: spawn bodies with **Q** using the on-screen spawner panel
- Rigid body physics: box, sphere, capsule colliders
- Fixed timestep, sequential impulse solver, manifold contacts
- Sleep/wake logic, restitution, friction
- Visual polish: wireframes, axis markers, cube outlines
- Fullscreen toggle (Alt+Enter)
- Clean separation: math, physics, rendering modules

## Notes

- Physics engine is completely independent of rendering
- Rendering is separated in the `render/` module
- VS Code is configured with IntelliSense support via `.vscode/c_cpp_properties.json`
- Tasks are set up in `.vscode/tasks.json` for one-click build/run