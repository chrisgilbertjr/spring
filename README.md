# Spring game dynamics
A lightweight 2D rigid body physics engine for games.

#### Whats new: ####
* Joints Demo now complete for MSVC2013!
* UNIX projects are somewhat functional using CMake. static libs can be built.

#### Whats coming: ####
* UNIX builds are still being worked on. shared libs and demo builds will be coming soon.
* XCode projects planned to be worked on after UNIX.

# About:
Spring game dynamics is a small small physics engine used to simulate rigid bodies in 2D.  Its written in C, and will have builds for multiple platforms. I hope you find something useful or at least have some fun with spring!

# Features:
* Circle, convex polygon, and line segment collision primitives.
* Supports multiple shapes per rigid body, allowing for the creation of non-convex shapes that interact believably.
* Broadphase collision detection using an array based sweep and prune algorithm.
* Narrowphase collision detection done using the GJK algorithm to determine if two objects intersect.
* Expanding polytype algorithm used to extract contact information from GJK collision.
* Fast constraint solver using the sequential impulse algorithm.
* Nine different joint types that allow the creation of things such as vehicles, bridges and ragdolls
* Simple to use API with good plenty of documentation within the code (doxygen is in the works)

# Build
__MSVC:__ Visual studio 2013 projects/solutions for libraries are available. Demos will be up shortly. Building them should be straightforward. I will support more visual studio versions once linux and mac builds are setup.

__UNIX:__ The UNIX build system is in the works. I will using be Cmake, and will have a CMakeLists file up within a few days.

__MAC OSX:__ Mac will also be supported once the UNIX build is working. You can use Cmake if you like, but i will be adding an xcode project in the future. I will also have to update some OpenGL code in order for demos to be ran, so this might take a while longer.

# Demos
There are a few demos and more will be added over time. I will be adding some to showcase the functionality of the engine. Demos will be included in seperate MSVC solutions and/or build steps for other platforms. The demo API was not intended for realtime use. Because of this, the documentation is very sparse.

demo controls:
r: reset the demo
g: toggle gravity

# Documentation
At the moment, the only place to get documentation on the code is the code itself. I will be working on getting doxygen documentation setup and included with the project once builds for all platforms are complete.
