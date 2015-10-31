
#ifndef SP_ANGULAR_SPRING_JOINT_H
#define SP_ANGULAR_SPRING_JOINT_H

#include "spConstraint.h"

/// @defgroup spAngularSpringJoint spAngularSpringJoint
/// @{

/// an angular spring joint is like a spring, but with rotation.
/// objects behave like a spring when rotating.
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
SPRING_API void spAngularSpringJointInit(spAngularSpringJoint* joint, spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle);

/// allocate an angular spring on the heap
SPRING_API spAngularSpringJoint* spAngularSpringJointAlloc();

/// create an angular spring on the heap with 2 bodies, an invert rot variable, spring frequency/damping, and the rest angle of the bodies
SPRING_API spConstraint* spAngularSpringJointNew(spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle);

/// check if a constraint is a angular spring joint
SPRING_API spBool spConstraintIsAngularSpringJoint(spConstraint* constraint);

/// safely cast a constraint to an angular spring joint if its that type
SPRING_API spAngularSpringJoint* spConstraintCastAngularSpringJoint(spConstraint* constraint);

/// get the joints impulse
SPRING_API spFloat spAngularSpringJointGetImpulse(spConstraint* constraint);

/// get the angular springs rest angle
SPRING_API spFloat spAngularSpringJointGetRestAngle(spConstraint* constraint);

/// get the angular springs frequency
SPRING_API spFloat spAngularSpringJointGetFrequency(spConstraint* constraint);

/// get the angular springs damping
SPRING_API spFloat spAngularSpringJointGetDamping(spConstraint* constraint);

/// get the angular springs inverse rot flag
SPRING_API spBool spAngularSpringJointGetInverse(spConstraint* constraint);

/// set the angular springs rest angle
SPRING_API void spAngularSpringJointSetRestAngle(spConstraint* constraint, spFloat restAngle);

/// set the angular springs frequency
SPRING_API void spAngularSpringJointSetFrequency(spConstraint* constraint, spFloat frequency);

/// set the angular springs damping
SPRING_API void spAngularSpringJointSetDamping(spConstraint* constraint, spFloat damping);

/// set the angular springs inverse rot flag
SPRING_API void spAngularSpringJointSetInverse(spConstraint* constraint, spBool inverse);

/// @}

#endif