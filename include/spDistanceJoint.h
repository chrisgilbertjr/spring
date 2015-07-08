
#ifndef SP_DISTANCE_CONSTRAINT_H
#define SP_DISTANCE_CONSTRAINT_H

#include "spConstraint.h"
#include "spMath.h"

/// @defgroup spDistanceJoint spDistanceJoint
/// @{

/// a distance joint hold two anchor points on two bodies at a set distance
struct spDistanceJoint
{
    spConstraint constraint; ///< base constraint class
    spVector anchorA;        ///< first anchor in body A's local space
    spVector anchorB;        ///< second anchor in body B's local space
    spVector rA;             ///< rel velocity of body A
    spVector rB;             ///< rel velocity of body B
    spVector n;              ///< impulse normal direction
    spFloat lambdaAccum;     ///< lagrange multiplier
    spFloat distance;        ///< distance between the anchor points
    spFloat eMass;           ///< effective mass
    spFloat bias;            ///< baumgarte velocity bias
};

/// init the distance joint given 2 bodies, 2 anchor points in local space, and a distance
void spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance);

/// allocate a distance joint on the heap
spDistanceJoint* spDistanceConstraintAlloc();

/// create a distance joint on the heap given 2 bodies, 2 anchor points in local space, and a distance
spDistanceJoint* spDistanceConstraintNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance);

/// free a distance joint from the heap
void spDistanceJointFree(spDistanceJoint** Joint);

/// setup a distance joint to be solved
void spDistanceJointPreSolve(spDistanceJoint* joint, const spFloat h);

/// solve the distance joint and apply the impulses
void spDistanceJointSolve(spDistanceJoint* joint);

/// check if a constraint is a distance joint
spBool spConstraintIsDistanceJoint(spConstraint* constraint);

/// safely cast a constraint to a distance joint if its that type
spDistanceJoint* spConstraintCastDistanceJoint(spConstraint* constraint);

/// get the distance joints first anchor in body A's local space
spVector spDistanceJointGetAnchorA(spConstraint* constraint);

/// get the distance joints second anchor in body B's local space
spVector spDistanceJointGetAnchorB(spConstraint* constraint);

/// get the distance joints first anchor in world space
spVector spDistanceJointWorldGetAnchorA(spConstraint* constraint);

/// get the distance joints second anchor in world space
spVector spDistanceJointWorldGetAnchorB(spConstraint* constraint);

/// get the distance joints distance between anchor points
spFloat spDistanceJointGetDistance(spConstraint* constraint);

/// set the distance joints first anchor in body A's local space
void spDistanceJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the distance joints second anchor in body B's local space
void spDistanceJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the distance joints first anchor in world space
void spDistanceJointWorldSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the distance joints second anchor in world space
void spDistanceJointWorldSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the distance joints distance between anchor points
void spDistanceJointSetDistance(spConstraint* constraint, spFloat distance);

/// @}

#endif