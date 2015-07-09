
#ifndef SP_WORLD_H
#define SP_WORLD_H

#include "spAngularSpringJoint.h"
#include "spDistanceJoint.h"
#include "spSpringJoint.h"
#include "spMotorJoint.h"
#include "spConstraint.h"
#include "spWheelJoint.h"
#include "spPointJoint.h"
#include "spMouseJoint.h"
#include "spGearJoint.h"
#include "spRopeJoint.h"
#include "spCollision.h"
#include "spSegment.h"
#include "spContact.h"
#include "spBody.h"

/// @defgroup spWorld spWorld
/// @{

/// a world is a collection of bodies, constraints, and contacts
/// a world is the home to the physics simulation
struct spWorld
{
    spConstraint* jointList; ///< list of active constraints
    spContact* contactList;  ///< list of active contacts
    spBody* bodyList;        ///< list of active bodies
    spVector gravity;        ///< world gravity
    spInt iterations;        ///< solver iterations
};

/// initialize a world
void spWorldInit(spWorld* world, spInt iterations, spVector gravity);

/// frees all memory (bodies, joints, contacts) that the world is simulating
void spWorldDestroy(spWorld* world);

/// construct and initialize a world
spWorld spWorldConstruct(spInt interations, spVector gravity);

/// step through the simulation.
void spWorldStep(spWorld* world, const spFloat dt);

// do broad phase collision detection
void spWorldBroadPhase(spWorld* world);

/// do narrow phase collision detection
void spWorldNarrowPhase(spWorld* world);

/// test a point against all shapes in the world. the first one is returned
spShape* spWorldTestPoint(spWorld* world, spVector point);

/// add a body to the world
void spWorldAddBody(spWorld* world, spBody* body);

/// remove a body from the world
void spWorldRemoveBody(spWorld* world, spBody* body);

/// add a constraint to the world
void spWorldAddConstraint(spWorld* world, spConstraint* constraint);

/// remove a constraint from the world
void spWorldRemoveConstraint(spWorld* world, spConstraint* constraint);

/// TEMP: draw the world with OPENGL1
void spWorldDraw(spWorld* world);

/// get the worlds joint list
spConstraint* spWorldGetJointList(spWorld* world);

/// get the worlds contact list
spContact* spWorldGetContactList(spWorld* world);

/// get the worlds body list
spBody* spWorldGetBodyList(spWorld* world);

/// get the worlds gravity
spVector spWorldGetGravity(spWorld* world);

/// get the worlds number of solver iterations
spInt spWorldGetIterations(spWorld* world);

/// set the worlds gravity
void spWorldSetGravity(spWorld* world, spVector gravity);

/// set the worlds solver iterations
void spWorldSetIterations(spWorld* world, spInt iterations);

/// @}

#endif