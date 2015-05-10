
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
spBool spNarrowPhaseCollide(const spNarrowPhase* narrow, spContact* contact, const spCollisionInput& data);

/// simulate one timestep of narrow phase collision detection
void spNarrowPhaseStep(spNarrowPhase* narrow, spContact*& contact_list);

/// @}

#endif