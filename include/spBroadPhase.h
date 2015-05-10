
#ifndef SP_BROAD_PHASE_H
#define SP_BROAD_PHASE_H

#include "spContact.h"

/// broad phase collision detection
struct spBroadPhase
{
    /// TODO: use a red-black tree for contact graph
    spContact* contact_list;
    spShape** shape_list;
};

/// 'faked' constructor for stack allocation
spBroadPhase _spBroadPhase(spShape* shape_list);

/// simulate one timestep of broad phase collision detection
void spBroadPhaseStep(spBroadPhase* broad, spBody* body_list, spContact*& contact_list);

#endif
