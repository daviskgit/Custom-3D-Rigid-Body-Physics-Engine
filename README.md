# Rigid-Body Physics Engine

A standalone 3D rigid-body physics engine written from scratch in modern C++,
with an OpenGL 4.5 rendering pipeline. No third-party dependencies (no GLFW,
GLEW/GLAD, GLM, Bullet, etc.) — windowing/GL context creation uses raw
Win32 + WGL, and math/collision/dynamics are all hand-rolled.

## Features

- **Fixed-rate simulation** decoupled from render rate: physics steps at a
  constant 120Hz via an accumulator, independent of frame time (see the loop
  in [src/main.cpp](src/main.cpp)).
- **GJK** ([src/collision.cpp](src/collision.cpp)) for boolean convex overlap
  detection on the Minkowski difference.
- **EPA** for penetration depth and contact normal once GJK finds overlap.
- **Contact manifold generation** via reference/incident face clipping
  (Sutherland–Hodgman), producing up to 4 stable contact points per pair
  instead of a single EPA point — this is what makes box stacks rest instead
  of jittering.
- **Impulse-based sequential solver** ([src/dynamics.cpp](src/dynamics.cpp)):
  resting contact via clamped non-negative normal impulses, Coulomb friction
  (two tangent directions, clamped to `μ * normalImpulse`), Baumgarte
  position-error stabilization, a restitution bias, and warm starting
  (impulses persisted per contact feature across frames).
- **Rotational dynamics**: quaternion orientation, semi-implicit Euler
  integration, world-space inverse inertia tensor recomputed from `R * I⁻¹ * Rᵀ`.
- **3D spatial hash broadphase** ([src/broadphase.cpp](src/broadphase.cpp)):
  bodies are rasterized into a uniform grid of cells; only bodies sharing a
  cell are tested in the narrowphase, taking the naive O(N²) pair sweep down
  to roughly O(N) for scenes with bounded local density.

## Project layout

```
src/
  math.h         vec3 / mat3 / quaternion / mat4 (hand-rolled)
  shapes.h       convex hull + support function, box construction
  collision.h/.cpp   GJK, EPA, face-clipping manifold generation
  broadphase.h/.cpp  uniform spatial hash
  dynamics.h/.cpp    RigidBody, World (integration + solver)
  gl_core.h      hand-declared OpenGL 4.5 function pointers (no GLAD/GLEW)
  renderer.h/.cpp    Win32 window, WGL 4.5 core context, instanced cube draw
  main.cpp       demo scene + fixed-timestep loop + orbit camera
```

## Building

Requires a C++17 compiler. Tested against the g++ shipped with MSYS2's
UCRT64 toolchain (`C:\msys64\ucrt64\bin\g++.exe`).

**Plain cmd/PowerShell, no MSYS shell needed:**
```
build.bat
bin\physics_engine.exe
```

**From an MSYS2 UCRT64 shell (or any shell with `mingw32-make` on PATH):**
```
mingw32-make run
```

There is no CMake dependency — both paths call g++ directly and statically
link the C/C++ runtime, so `bin\physics_engine.exe` runs standalone.

## Controls

- **Left-click drag** — orbit the camera
- **Scroll wheel** — zoom
- **Esc** — quit

The window title shows live diagnostics: body count, broadphase candidate
pairs, and resolved narrowphase contacts.

## Demo scene

A 3×8 box stack (friction + resting-contact stress test) plus 40 randomly
sized, randomly oriented tumbling boxes dropped from height onto a static
ground slab — enough simultaneous pairs to exercise the spatial hash and the
manifold solver together.
