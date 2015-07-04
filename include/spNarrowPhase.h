
#ifndef SP_NARROW_PHASE_H
#define SP_NARROW_PHASE_H

#include "spCollision.h"
#include "spShape.h"

/// @defgroup spNarrowPhase spNarrowPhase
/// @{

struct spNarrowPhase
{
    spCollisionMatrix collision_matrix;
    spContact** contact_list;
};

/// 'faked' constructor for stack allocation
spNarrowPhase _spNarrowPhase(spContact* contact_list);

/// collide two shapes
spCollisionResult spNarrowPhaseCollide(const spNarrowPhase* narrow, const spShape* shapeA, const spShape* shapeB, spContact* contact);

/// simulate one timestep of narrow phase collision detection
void spNarrowPhaseStep(spNarrowPhase* narrow, spContact*& contact_list);

/// @}

#endif