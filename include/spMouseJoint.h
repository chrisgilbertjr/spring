
#ifndef SP_MOUSE_JOINT_H
#define SP_MOUSE_JOINT_H

#include "spConstraint.h"
#include "spBody.h"
#include "spMath.h"

/// @defgroup spMouseJoint
/// @{

/// a mouse joint is a soft spring constraint, where one point is a desired world position of the other body point
/// the body is 'dragged' by a mouse from a starting position to an ending position. this position should be updated
/// every frame. the mouse joint only affects one body at a time
struct spMouseJoint
{
    spConstraint constraint; ///< base constraint class
    spVector lambdaAccum;    ///< accumulated lagrange multiplier
    spVector anchor;         ///< the anchor point on body A in local space
    spVector target;         ///< the target position of the anchor in world space
    spVector bias;           ///< baumgarte velocity bias
    spVector rA;             ///< rel velocity from anchor to body A com
    spFloat frequency;       ///< spring oscillation per second
    spFloat damping;         ///< spring damping. 1 is not damped. 0 is completely damped
    spFloat gamma;           ///< temp variable for solving soft springs
    spMatrix eMass;          ///< effective mass
};

/// init a mouse joint given a body, frequency, damping, anchor point, and target point
void spMouseJointInit(spMouseJoint* joint, spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target);

/// allocate memory for a mouse joint on the heap
spMouseJoint* spMouseJointAlloc();

/// create a mouse joint on the heap given a body, frequency, damping, anchor point, and target point
spMouseJoint* spMouseJointNew(spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target);

/// check if a constraint is a mouse joint
spBool spConstraintIsMouseJoint(spConstraint* constraint);

/// safely cast a constraint to a gear joint if its that type
spMouseJoint* spConstraintCastMouseJoint(spConstraint* constraint);

/// start a mouse drag on body a given a world anchor point
void spMouseJointStart(spConstraint* constraint, spBody* a, spVector point);

/// end a mouse drag 
void spMouseJointEnd(spConstraint* constraint);

/// get the mouse joints impulse
spVector spMouseJointGetImpulse(spConstraint* constraint);

/// get the body anchor in local space
spVector spMouseJointGetAnchor(spConstraint* constraint);

/// get the body anchor in world space
spVector spMouseJointGetWorldAnchor(spConstraint* constraint);

/// get the target in world space
spVector spMouseJointGetTarget(spConstraint* constraint);

/// get the target in body A's local space
spVector spMouseJointGetLocalTarget(spConstraint* constraint);

/// get the spring frequency of the mouse constraint
spFloat spMouseJointGetFrequency(spConstraint* constraint);

/// get the spring damping of the mouse joint
spFloat spMouseJointGetDamping(spConstraint* constraint);

/// set the mouse joints anchor in body A's local space
void spMouseJointSetAnchor(spConstraint* constraint, spVector anchor);

/// set the mouse joints anchor in world space
void spMouseJointSetWorldAnchor(spConstraint* constraint, spVector anchor);

/// set the mouse joints target with a world space point
void spMouseJointSetTarget(spConstraint* constraint, spVector target);

/// set the mouse joints target with a point in body A's local space
void spMouseJointSetLocalTarget(spConstraint* constraint, spVector target);

/// set the spring frequency of the mouse joint
void spMouseJointSetFrequency(spConstraint* constraint, spFloat frequency);

/// set the spring damping of the mouse joint
void spMouseJointSetDamping(spConstraint* constraint, spFloat damping);

/// @}

#endif