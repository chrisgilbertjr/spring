# Spring game dynamics
A lightweight 2D rigid body physics engine for games.

# About:
Spring game dynamics is a small small physics engine used to simulate rigid bodies in 2D.  Its written in C, and compiles on MSVC13 at the moment (linux builds are in the works, check back soon).

# Features:
* Circle, convex polygon, and line segment collision primitives.
* Supports multiple shapes per rigid body, allowing for the creation of non-convex shapes that interact believably.
* Broadphase collision detection using an array based sweep and prune algorithm.
* Narrowphase collision detection done using the GJK algorithm to determine if two objects intersect.
* Expanding polytype algorithm used to extract contact information from GJK collision.
* Fast constraint solver based on Erin Catto's sequential impulses algorithm.
* Nine different joint types that allow the creation of things such as vehicles, bridges and ragdolls
* Simple to use API with good plenty of documentation within the code (doxygen is in the works)

# Build
__MSVC:__ Visual studio 2013 projects/solutions for libraries and demos are available. Building them should be straightforward. I will support more visual studio versions once linux and mac builds are setup.

__UNIX:__ The UNIX build system is in the works. I will using be Cmake, and will have a CMakeLists file up within a few days.

__MAC OSX:__ Mac will also be supported once the UNIX build is working. You can use Cmake if you like, but i will be adding an xcode project in the future. I will also have to update some OpenGL code in order for demos to be ran, so this might take a while longer.
