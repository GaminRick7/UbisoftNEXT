# BAGGAGE | Ubisoft NEXT 2026 Finalist

https://github.com/user-attachments/assets/85848714-5eab-44ad-8b77-97dbc2beee93

I placed 2nd at the Ubisoft NEXT 2026 Programming Contest!

This year's theme was all about **simulating a large number of entities**. The contest involved building an entire game engine from scratch in C++ on top of the Ubisoft API (OpenGL/FreeGLUT), and then building a game on top of it.

**BAGGAGE** is my take on that: a crowd simulation puzzle game where you place directional arrows on a height-mapped grid to shepherd thousands of agents toward a goal, all while drones patrol the area trying to pick them off. 

## Features

I wrote a full game engine from scratch for this contest. Some highlights:

- A full **ECS** (entity-component-system) from scratch, with bitmask-based component queries, deferred entity destruction, typed entity views, and support for thousands of active entities
- **Custom rendering pipeline** with MVP transforms, point lighting, and OBJ model loading
- **Flow field pathfinding** combined with flocking behavior (separation/cohesion) for **2000+ agents at 60 FPS**
- **Spatial hashing** for crowd neighbor lookups, replacing brute-force O(n^2) checks and keeping the simulation fast
- **Octree-based spatial partitioning** for frustum culling, AABB, sphere, and ray queries, making sure only visible geometry gets touched
- **Sparse voxel octree (SVON)** built from static colliders for efficient 3D pathfinding and navigation using **A***
- **Rigidbody physics** with gravity, sleep, and collision events
- A **scene system** with a main menu, the main gameplay level, drone AI, a camera flyby intro, and a win condition
- **Time scale control** (0.5x to 2x) via controller triggers

## Building

**Windows:** Visual Studio 2022, CMake 3.29+

**macOS:** Xcode CLI tools, CMake, Homebrew, `brew install freeglut sdl3`

Windows:
```
generate-windows.bat
# Open build/win64 in Visual Studio, build and run
```

macOS:
```
./generate-macos.sh
cd build/macos
make all
make run
```

Use Cmd+Q to quit on macOS.

## Controls

Gamepad recommended (Xbox layout). Place flow arrows on the grid to direct agents. Left/right triggers control time scale.
