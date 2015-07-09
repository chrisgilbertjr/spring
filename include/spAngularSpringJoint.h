
#ifndef SP_ANGULAR_SPRING_JOINT_H
#define SP_ANGULAR_SPRING_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

/// @defgroup spAngularSpringJoint spAngularSpringJoint
/// @{

struct spAngularSpringJoint
{
    spConstraint constraint; ///< base constraint class
    spFloat lambdaAccum;     ///< lagrange multiplier
    spFloat restAngle;       ///< rest angle between the two bodies
    spFloat frequency;       ///< spring frequency
    spFloat damping;         ///< spring damping
    spFloat eMass;           ///< effective mass
    spFloat gamma;           ///< temp solver
    spFloat bias;            ///< baumgarte velocity bias
    spBool inverse;          ///< true to reverse rotation angle
};

/// init an angular spring with 2 bodies, an invert rot variable, spring frequency/damping, and the rest angle of the bodies
void spAngularSpringJointInit(spAngularSpringJoint* joint, spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle);

/// allocate an angular spring on the heap
spAngularSpringJoint* spAngularSpringJointAlloc();

/// create an angular spring on the heap with 2 bodies, an invert rot variable, spring frequency/damping, and the rest angle of the bodies
spConstraint* spAngularSpringJointNew(spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle);

/// free an angular spring on the heap
void spAngularSpringJointFree(spAngularSpringJoint** joint);

/// setup the angular spring to be solved
void spAngularSpringJointPreSolve(spAngularSpringJoint* joint, const spFloat h);

/// warm start spring joint from last timestep
void spAngularSpringJointApplyCachedImpulse(spAngularSpringJoint* joint);

/// solve the angular spring and apply the impulses
void spAngularSpringJointSolve(spAngularSpringJoint* joint);

/// check if a constraint is a angular spring joint
spBool spConstraintIsAngularSpringJoint(spConstraint* constraint);

/// safely cast a constraint to an angular spring joint if its that type
spAngularSpringJoint* spConstraintCastAngularSpringJoint(spConstraint* constraint);

/// get the angular springs rest angle
spFloat spAngularSpringJointGetRestAngle(spConstraint* constraint);

/// get the angular springs frequency
spFloat spAngularSpringJointGetFrequency(spConstraint* constraint);

/// get the angular springs damping
spFloat spAngularSpringJointGetDamping(spConstraint* constraint);

/// get the angular springs inverse rot flag
spBool spAngularSpringJointGetInverse(spConstraint* constraint);

/// set the angular springs rest angle
void spAngularSpringJointSetRestAngle(spConstraint* constraint, spFloat restAngle);

/// set the angular springs frequency
void spAngularSpringJointSetFrequency(spConstraint* constraint, spFloat frequency);

/// set the angular springs damping
void spAngularSpringJointSetDamping(spConstraint* constraint, spFloat damping);

/// set the angular springs inverse rot flag
void spAngularSpringJointSetInverse(spConstraint* constraint, spBool inverse);

/// @}

#endif