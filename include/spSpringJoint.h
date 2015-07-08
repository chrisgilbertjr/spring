
#ifndef SP_SPRING_JOINT_H
#define SP_SPRING_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

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
    spFloat beta;            /// temp solver
};

/// init a spring joint with 2 bodies, 2 anchor points, damping, frequency, and damping.
void spSpringJointInit(spSpringJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat frequency, spFloat damping, spFloat restLength);

/// allocate a spring joint on the heap
spSpringJoint* spSpringJointAlloc();

/// create a spring joint on the heap with 2 bodies, 2 anchor points, damping, frequency, and damping.
spConstraint* spSpringJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat frequency, spFloat damping, spFloat restLength);

/// release the memory pointed at by a spring joint
void spSpringJointFree(spSpringJoint** joint);

/// setup a spring joint to be solved by the solver
void spSpringJointPreStep(spSpringJoint* joint, const spFloat h);

/// solve and apply impulses to a spring joint 
void spSpringJointSolve(spSpringJoint* joint);

/// check if a constraint is a spring joint
spBool spConstraintIsSpringJoint(spConstraint* constraint);

/// safely cast a constraint to a spring joint if its that type
spSpringJoint* spConstraintCastSpringJoint(spConstraint* constraint);

/// get the spring joints first anchor in body A's local space
spVector spSpringJointGetAnchorA(spConstraint* constraint);

/// get the spring joints second anchor in body B's local space
spVector spSpringJointGetAnchorB(spConstraint* constraint);

/// get the spring joints first anchor in world space
spVector spSpringJointGetWorldAnchorA(spConstraint* constraint);

/// get the spring joints second anchor in world space
spVector spSpringJointGetWorldAnchorB(spConstraint* constraint);

/// get the spring joints rest length
spFloat spSpringJointGetRestLength(spConstraint* constraint);

/// get the spring joints frequency
spFloat spSpringJointGetFrequency(spConstraint* constraint);

/// get the spring joints damping
spFloat spSpringJointGetDamping(spConstraint* constraint);

/// set the spring joints first anchor in body A's local space
void spSpringJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the spring joints second anchor in body B's local space
void spSpringJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the spring joints first anchor in world space
void spSpringJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set the spring joints second anchor in world space
void spSpringJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// set the spring joints rest length
void spSpringJointSetRestLength(spConstraint* constraint, spFloat restLength);

/// set the spring joints frequency
void spSpringJointSetFrequency(spConstraint* constraint, spFloat frequency);

/// set the spring joints damping
void spSpringJointSetDamping(spConstraint* constraint, spFloat damping);

/// @}

#endif