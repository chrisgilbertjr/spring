#### Spring game dynamics ####

Spring game dynamics is a small small physics engine used to simulate rigid bodies in 2D. Its written in C and does not use any external libraries. It can be built using MS Visual Studio, but UNIX builds using CMake are in the works. Spring was developed mainly for educational purposes, so I hope you can find it useful!

#### Features: ####
* Circle, convex polygon, and line segment collision primitives.
* Supports multiple shapes per rigid body, allowing for the creation of non-convex shapes that interact believably.
* Broadphase collision detection using an array based sweep and prune algorithm.
* Narrowphase collision detection done using the GJK algorithm to determine if two objects intersect.
* Expanding polytype algorithm used to extract contact information from GJK collision.
* Fast constraint solver using the sequential impulse algorithm.
* Eight different joint types that allow the creation of things such as vehicles, bridges and ragdolls

#### Demos ####
Demos in Spring are built using the included demo API, which is based on GLFW3 and require _OpenGL 3.3_. To run the demos, build the included Visual Studio solution msvc/vs2013/demos, and a list of demos will be available to run. UNIX builds will be available in the future. Below i will include a link to videos of each demo running.

* Joints  - https://www.youtube.com/watch?v=08gRirQtBT4 
* Concave - https://www.youtube.com/watch?v=ZiJPUX2PrFM 
* Chains  - https://www.youtube.com/watch?v=HVK0ORYZ0H0 
* Cloth   - https://www.youtube.com/watch?v=rSIz-SLTQGA
