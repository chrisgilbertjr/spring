
#ifndef SP_WORLD_H
#define SP_WORLD_H

#include "spSweepAndPrune.h"
#include "spMath.h"

/// forward declarations to reduce includes
struct spConstraint;
struct spContact;
struct spBody;

/// @defgroup spWorld spWorld
/// @{

/// a world is a collection of bodies, constraints, and contacts
/// a world is the home to the physics simulation
struct spWorld
{
    spConstraint* jointList; ///< list of active constraints
    spContact* contactList;  ///< list of active contacts
    spBody* bodyList;        ///< list of active bodies
    spSap sweepAndPrune;     ///< sweep and prune broadphase
    spVector gravity;        ///< world gravity
    spInt iterations;        ///< solver iterations
};

/// initialize a world
SPRING_API void spWorldInit(spWorld* world, spInt iterations, spVector gravity);

/// frees all memory (bodies, joints, contacts) that the world is simulating
SPRING_API void spWorldDestroy(spWorld* world);

/// construct and initialize a world
SPRING_API spWorld spWorldConstruct(spInt interations, spVector gravity);

/// step through the simulation.
SPRING_API void spWorldStep(spWorld* world, const spFloat dt);

// do broad phase collision detection using sweep and prune
SPRING_API void spWorldBroadPhaseSAP(spWorld* world);

// do broad phase collision detection using brute force
SPRING_API void spWorldBroadPhaseBruteForce(spWorld* world);

/// do narrow phase collision detection
SPRING_API void spWorldNarrowPhase(spWorld* world);

/// test a point against all shapes in the world. the first one is returned
SPRING_API spShape* spWorldTestPoint(spWorld* world, spVector point);

/// add a body to the world
SPRING_API void spWorldAddBody(spWorld* world, spBody* body);

/// remove a body from the world
SPRING_API void spWorldRemoveBody(spWorld* world, spBody* body);

/// add a constraint to the world
SPRING_API void spWorldAddConstraint(spWorld* world, spConstraint* constraint);

/// remove a constraint from the world
SPRING_API void spWorldRemoveConstraint(spWorld* world, spConstraint* constraint);

/// get the worlds joint list
SPRING_API spConstraint* spWorldGetJointList(spWorld* world);

/// get the worlds contact list
SPRING_API spContact* spWorldGetContactList(spWorld* world);

/// get the worlds body list
SPRING_API spBody* spWorldGetBodyList(spWorld* world);

/// get the worlds gravity
SPRING_API spVector spWorldGetGravity(spWorld* world);

/// get the worlds number of solver iterations
SPRING_API spInt spWorldGetIterations(spWorld* world);

/// set the worlds gravity
SPRING_API void spWorldSetGravity(spWorld* world, spVector gravity);

/// set the worlds solver iterations
SPRING_API void spWorldSetIterations(spWorld* world, spInt iterations);

/// @}

#endif