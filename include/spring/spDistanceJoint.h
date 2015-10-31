
#ifndef SP_DISTANCE_CONSTRAINT_H
#define SP_DISTANCE_CONSTRAINT_H

#include "spConstraint.h"

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
SPRING_API void spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance);

/// allocate a distance joint on the heap
SPRING_API spDistanceJoint* spDistanceJointAlloc();

/// create a distance joint on the heap given 2 bodies, 2 anchor points in local space, and a distance
SPRING_API spConstraint* spDistanceJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance);

/// check if a constraint is a distance joint
SPRING_API spBool spConstraintIsDistanceJoint(spConstraint* constraint);

/// safely cast a constraint to a distance joint if its that type
SPRING_API spDistanceJoint* spConstraintCastDistanceJoint(spConstraint* constraint);

/// get the distance joints impulse
SPRING_API spFloat spDistanceJointGetImpulse(spConstraint* constraint);

/// get the distance joints first anchor in body A's local space
SPRING_API spVector spDistanceJointGetAnchorA(spConstraint* constraint);

/// get the distance joints second anchor in body B's local space
SPRING_API spVector spDistanceJointGetAnchorB(spConstraint* constraint);

/// get the distance joints first anchor in world space
SPRING_API spVector spDistanceJointGetWorldAnchorA(spConstraint* constraint);

/// get the distance joints second anchor in world space
SPRING_API spVector spDistanceJointGetWorldAnchorB(spConstraint* constraint);

/// get the distance joints distance between anchor points
SPRING_API spFloat spDistanceJointGetDistance(spConstraint* constraint);

/// set the distance joints first anchor in body A's local space
SPRING_API void spDistanceJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the distance joints second anchor in body B's local space
SPRING_API void spDistanceJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the distance joints first anchor in world space
SPRING_API void spDistanceJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set the distance joints second anchor in world space
SPRING_API void spDistanceJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// set the distance joints distance between anchor points
SPRING_API void spDistanceJointSetDistance(spConstraint* constraint, spFloat distance);

/// @}

#endif