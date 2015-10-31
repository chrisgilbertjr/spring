
#ifndef SP_SPRING_JOINT_H
#define SP_SPRING_JOINT_H

#include "spConstraint.h"

/// @defgroup spSpringJoint spSpringJoint
/// @{

/// a spring joint is a soft distance constraint
struct spSpringJoint
{
    spConstraint constraint; /// constraint base class
    spVector anchorA;        /// local anchor on body A's local space
    spVector anchorB;        /// local anchor on body B's local space
    spVector rA;             /// relative velocity of body A
    spVector rB;             /// relative velocity of body B
    spVector n;              /// normal impulse direction
    spFloat lambdaAccum;     /// accumulated lagrange multiplier
    spFloat restLength;      /// rest length of the spring
    spFloat frequency;       /// frequency of oscilation per second
    spFloat damping;         /// springs damping
    spFloat eMass;           /// effective mass
    spFloat gamma;           /// temp solver
    spFloat bias;            /// baumgarte veloicity bias
};

/// init a spring joint with 2 bodies, 2 anchor points, damping, frequency, and damping.
SPRING_API void spSpringJointInit(spSpringJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat frequency, spFloat damping, spFloat restLength);

/// allocate a spring joint on the heap
SPRING_API spSpringJoint* spSpringJointAlloc();

/// create a spring joint on the heap with 2 bodies, 2 anchor points, damping, frequency, and damping.
SPRING_API spConstraint* spSpringJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat frequency, spFloat damping, spFloat restLength);

/// check if a constraint is a spring joint
SPRING_API spBool spConstraintIsSpringJoint(spConstraint* constraint);

/// safely cast a constraint to a spring joint if its that type
SPRING_API spSpringJoint* spConstraintCastSpringJoint(spConstraint* constraint);

/// get the spring joints impulse
SPRING_API spFloat spSpringJointGetImpulse(spConstraint* constraint);

/// get the spring joints first anchor in body A's local space
SPRING_API spVector spSpringJointGetAnchorA(spConstraint* constraint);

/// get the spring joints second anchor in body B's local space
SPRING_API spVector spSpringJointGetAnchorB(spConstraint* constraint);

/// get the spring joints first anchor in world space
SPRING_API spVector spSpringJointGetWorldAnchorA(spConstraint* constraint);

/// get the spring joints second anchor in world space
SPRING_API spVector spSpringJointGetWorldAnchorB(spConstraint* constraint);

/// get the spring joints rest length
SPRING_API spFloat spSpringJointGetRestLength(spConstraint* constraint);

/// get the spring joints frequency
SPRING_API spFloat spSpringJointGetFrequency(spConstraint* constraint);

/// get the spring joints damping
SPRING_API spFloat spSpringJointGetDamping(spConstraint* constraint);

/// set the spring joints first anchor in body A's local space
SPRING_API void spSpringJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the spring joints second anchor in body B's local space
SPRING_API void spSpringJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the spring joints first anchor in world space
SPRING_API void spSpringJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set the spring joints second anchor in world space
SPRING_API void spSpringJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// set the spring joints rest length
SPRING_API void spSpringJointSetRestLength(spConstraint* constraint, spFloat restLength);

/// set the spring joints frequency
SPRING_API void spSpringJointSetFrequency(spConstraint* constraint, spFloat frequency);

/// set the spring joints damping
SPRING_API void spSpringJointSetDamping(spConstraint* constraint, spFloat damping);

/// @}

#endif