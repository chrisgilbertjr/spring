
#ifndef SP_GEAR_JOINT_H
#define SP_GEAR_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

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
void spGearJointInit(spGearJoint* joint, spBody* a, spBody* b, spFloat ratio, spFloat phase);

/// allocate memory for a gear joint on the heap
spGearJoint* spGearJointAlloc();

/// create a gear joint with two bodies, an angular ratio, and a phase angle
spConstraint* spGearJointNew(spBody* a, spBody* b, spFloat ratio, spFloat phase);

/// free a gear joint from the heap
void spGearJointFree(spGearJoint** joint);

/// warm start the gear joint and apply last frames cached impulse
void spGearJointApplyCachedImpulses(spGearJoint* joint);

/// setup the gear joint to be solved
void spGearJointPreSolve(spGearJoint* joint, const spFloat h);

/// solve the gear joint and apply the impulses
void spGearJointSolve(spGearJoint* joint);


/// check if a constraint is a gear joint
spBool spConstraintIsGearJoint(spConstraint* constraint);

/// safely cast a constraint to a gear joint if its that type
spGearJoint* spConstraintCastGearJoint(spConstraint* constraint);

/// get the angular impulse used on both bodies (this is not the applied impulse)
spFloat spGearJointGetImpulse(spGearJoint* joint);

/// get the gear ratio
spFloat spGearGetRatio(spGearJoint* joint);

/// get the gear phase angle
spFloat spGearGetPhase(spGearJoint* joint);

/// set the gear ratio
void spGearSetRatio(spGearJoint* joint, spFloat ratio);

/// set the gear phase angle
void spGearSetPhase(spGearJoint* joint, spFloat phase);

/// @}

#endif