
#ifndef SP_POINT_JOINT_H
#define SP_POINT_JOINT_H

#include "spConstraint.h"

/// @defgroup spPointJoint spPointJoint
/// @{

/// a point joint limits two bodies to rotate freely around two local space points
/// this joint can act as a hinge/revolute joint
struct spPointJoint
{
    spConstraint constraint; ///< base constraint class
    spVector anchorA;        ///< first anchor in body A's local space
    spVector anchorB;        ///< second anchor in body B's local space
    spVector lambdaAccum;    ///< accumulated lagrange multiplier
    spVector bias;           ///< baumgarte velocity bias
    spVector rA;             ///< rel velocity of body A
    spVector rB;             ///< rel velocity of body B
    spMatrix eMass;          ///< effective mass matrix
};

/// init a point joint with 2 bodies, and 2 local space anchors
void spPointJointInit(spPointJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB);

/// allocate a point joint on the heap
spPointJoint* spPointJointAlloc();

/// create a point joint on the heap from 2 bodies and 2 local space anchors
spPointJoint* spPointJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB);

/// check if a constraint is a point joint
spBool spConstraintIsPointJoint(spConstraint* constraint);

/// safely cast a constraint to a point joint if its that type
spPointJoint* spConstraintCastPointJoint(spConstraint* constraint);

/// get the point joints impulse
spVector spPointJointGetImpulse(spConstraint* constraint);

/// get the point joints first anchor in body A's local space
spVector spPointJointGetAnchorA(spConstraint* constraint);

/// get the point joints second anchor in body B's local space
spVector spPointJointGetAnchorB(spConstraint* constraint);

/// get the point joints first anchor in world space
spVector spPointJointGetWorldAnchorA(spConstraint* constraint);

/// get the point joints second anchor in world space
spVector spPointJointGetWorldAnchorB(spConstraint* constraint);

/// get the last impulse applied to body A
spVector spPointJointGetImpulseA(spConstraint* constraint);

/// get the last impulse applied to body B
spVector spPointJointGetImpulseB(spConstraint* constraint);

/// get the rel velocity from the point of rotation of body A
spVector spPointJointGetRelVelocityA(spConstraint* constraint);

/// get the rel velocity from the point of rotation of body B
spVector spPointJointGetRelVelocityB(spConstraint* constraint);

/// set the first anchor in body A's local space
void spPointJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the second anchor in body B's local space
void spPointJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the first anchor in world space
void spPointJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set the second anchor in world space
void spPointJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// @}

#endif