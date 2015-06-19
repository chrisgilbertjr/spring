
#ifndef SP_WORLD_H
#define SP_WORLD_H

#include "spConstraint.h"
#include "spContact.h"
#include "spNarrowPhase.h"
#include "spBroadPhase.h"
#include "spBody.h"
#include "spDistanceJoint.h"
#include "spRopeJoint.h"
#include "spMotorJoint.h"
#include "spSpringJoint.h"
#include "spAngularSpringJoint.h"
#include "spWheelJoint.h"
#include "spGearJoint.h"

/// @defgroup spWorld spWorld
/// @{

struct spWorld
{
    spBroadPhase broad_phase;
    spNarrowPhase narrow_phase;
    spConstraint* joint_list;
    spContact* contact_list;
    spBody* body_list;
    spVector gravity;
    spInt iterations;
};

/// initialize a world
void spWorldInit(spWorld* world, const spVector& gravity);

/// construct and initialize a world
spWorld _spWorld(const spVector& gravity);

/// step through the simulation.
void spWorldStep(spWorld* world, const spFloat dt);

/// gives a brief log of the world
void spWorldLogBrief(spWorld* world);

/// gives a brief log of the body list
void spWorldLogBriefBodies(spWorld* world);

/// gives a detailed log of the world
void spWorldLogDetail(spWorld* world);

/// TODO:
void spWorldAddDistanceJoint(spWorld* world, spDistanceJoint* joint);

/// TODO:
void spWorldAddRopeJoint(spWorld* world, spRopeJoint* joint);

// TODO:
void spWorldAddMotorJoint(spWorld* world, spMotorJoint* joint);

/// TODO:
void spWorldAddSpringJoint(spWorld* world, spSpringJoint* joint);

/// TODO:
void spWorldAddAngularSpringJoint(spWorld* world, spAngularSpringJoint* joint);

/// TODO:
void spWorldAddWheelJoint(spWorld* world, spWheelJoint* joint);

/// TODO:
void spWorldAddGearJoint(spWorld* world, spGearJoint* joint);

void spWorldDraw(spWorld* world);

/// sanity check
#ifdef SP_DEBUG
 inline void _spWorldIsSane(spWorld* world)
 {
     spAssert(world != NULL, "world is null in sanity check!");
 }
 #define spWorldIsSane(world) _spWorldIsSane(world)
#else
 #define spWorldIsSane(world)
#endif

/// @}

#endif