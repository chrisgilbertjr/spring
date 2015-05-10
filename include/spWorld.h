
#ifndef SP_WORLD_H
#define SP_WORLD_H

#include "spConstraint.h"
#include "spContact.h"
#include "spCluster.h"
#include "spNarrowPhase.h"
#include "spBroadPhase.h"
#include "spBody.h"

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