
#ifndef SP_WORLD_H
#define SP_WORLD_H

#include "spConstraint.h"
#include "spContact.h"
#include "spBody.h"
#include "spDistanceJoint.h"
#include "spRopeJoint.h"
#include "spMotorJoint.h"
#include "spSpringJoint.h"
#include "spAngularSpringJoint.h"
#include "spWheelJoint.h"
#include "spGearJoint.h"
#include "spPointJoint.h"
#include "spMouseJoint.h"
#include "spSegment.h"
#include "spCollision.h"

/// @defgroup spWorld spWorld
/// @{

struct spWorld
{
    spConstraint* joint_list;
    spContact* contact_list;
    spBody* body_list;
    spVector gravity;
    spInt iterations;
};

/// initialize a world
void spWorldInit(spWorld* world, const spVector& gravity);

/// construct and initialize a world
spWorld spWorldConstruct(const spVector& gravity);

/// step through the simulation.
void spWorldStep(spWorld* world, const spFloat dt);

// do broad phase collision detection
void spWorldBroadPhase(spWorld* world);

/// do narrow phase collision detection
void spWorldNarrowPhase(spWorld* world);

/// gives a brief log of the world
void spWorldLogBrief(spWorld* world);

/// gives a brief log of the body list
void spWorldLogBriefBodies(spWorld* world);

/// gives a detailed log of the world
void spWorldLogDetail(spWorld* world);

spShape* spWorldTestPoint(spWorld* world, spVector point);

/// TODO:
void spWorldAddBody(spWorld* world, spBody* body);

void spWorldAddConstraint(spWorld* world, spConstraint* constraint);

void spWorldRemoveConstraint(spWorld* world, spConstraint* constraint);

void spWorldDraw(spWorld* world);

/// @}

#endif