
#ifndef SP_GEAR_JOINT_H
#define SP_GEAR_JOINT_H

#include "spConstraint.h"

/// @defgroup spGearJoint spGearJoint
/// @{

/// a gear joint holds two bodies angular velocity at a constant ratio
struct spGearJoint
{
    spConstraint constraint; ///< base constraint class
    spFloat lambdaAccum;     ///< accumulated lagrange multiplier
    spFloat ratioInv;        ///< inverse ratio
    spFloat ratio;           ///< angular ratio between bodyA and bodyB
    spFloat eMass;           ///< effective mass of the constraint
    spFloat phase;           ///< phase difference between the two bodies
    spFloat bias;            ///< baumgarte velocity bias
};

/// initialize a gear joint with two bodies, an angular ratio, and a phase angle
SPRING_API void spGearJointInit(spGearJoint* joint, spBody* a, spBody* b, spFloat ratio, spFloat phase);

/// allocate memory for a gear joint on the heap
SPRING_API spGearJoint* spGearJointAlloc();

/// create a gear joint with two bodies, an angular ratio, and a phase angle
SPRING_API spConstraint* spGearJointNew(spBody* a, spBody* b, spFloat ratio, spFloat phase);

/// check if a constraint is a gear joint
SPRING_API spBool spConstraintIsGearJoint(spConstraint* constraint);

/// safely cast a constraint to a gear joint if its that type
SPRING_API spGearJoint* spConstraintCastGearJoint(spConstraint* constraint);

/// get the gear joints impulse
SPRING_API spFloat spGearJointGetImpulse(spConstraint* constraint);

/// get the gear ratio
SPRING_API spFloat spGearGetRatio(spConstraint* constraint);

/// get the gear phase angle
SPRING_API spFloat spGearGetPhase(spConstraint* constraint);

/// set the gear ratio
SPRING_API void spGearSetRatio(spConstraint* constraint, spFloat ratio);

/// set the gear phase angle
SPRING_API void spGearSetPhase(spConstraint* constraint, spFloat phase);

/// @}

#endif