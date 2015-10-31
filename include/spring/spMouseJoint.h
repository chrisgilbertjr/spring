
#ifndef SP_MOUSE_JOINT_H
#define SP_MOUSE_JOINT_H

#include "spConstraint.h"

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
SPRING_API void spMouseJointInit(spMouseJoint* joint, spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target);

/// allocate memory for a mouse joint on the heap
SPRING_API spMouseJoint* spMouseJointAlloc();

/// create a mouse joint on the heap given a body, frequency, damping, anchor point, and target point
SPRING_API spConstraint* spMouseJointNew(spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target);

/// check if a constraint is a mouse joint
SPRING_API spBool spConstraintIsMouseJoint(spConstraint* constraint);

/// safely cast a constraint to a gear joint if its that type
SPRING_API spMouseJoint* spConstraintCastMouseJoint(spConstraint* constraint);

/// start a mouse drag on body a given a world anchor point
SPRING_API void spMouseJointStart(spConstraint* constraint, spBody* a, spVector point);

/// end a mouse drag 
SPRING_API void spMouseJointEnd(spConstraint* constraint);

/// get the mouse joints impulse
SPRING_API spVector spMouseJointGetImpulse(spConstraint* constraint);

/// get the body anchor in local space
SPRING_API spVector spMouseJointGetAnchor(spConstraint* constraint);

/// get the body anchor in world space
SPRING_API spVector spMouseJointGetWorldAnchor(spConstraint* constraint);

/// get the target in world space
SPRING_API spVector spMouseJointGetTarget(spConstraint* constraint);

/// get the target in body A's local space
SPRING_API spVector spMouseJointGetLocalTarget(spConstraint* constraint);

/// get the spring frequency of the mouse constraint
SPRING_API spFloat spMouseJointGetFrequency(spConstraint* constraint);

/// get the spring damping of the mouse joint
SPRING_API spFloat spMouseJointGetDamping(spConstraint* constraint);

/// set the mouse joints anchor in body A's local space
SPRING_API void spMouseJointSetAnchor(spConstraint* constraint, spVector anchor);

/// set the mouse joints anchor in world space
SPRING_API void spMouseJointSetWorldAnchor(spConstraint* constraint, spVector anchor);

/// set the mouse joints target with a world space point
SPRING_API void spMouseJointSetTarget(spConstraint* constraint, spVector target);

/// set the mouse joints target with a point in body A's local space
SPRING_API void spMouseJointSetLocalTarget(spConstraint* constraint, spVector target);

/// set the spring frequency of the mouse joint
SPRING_API void spMouseJointSetFrequency(spConstraint* constraint, spFloat frequency);

/// set the spring damping of the mouse joint
SPRING_API void spMouseJointSetDamping(spConstraint* constraint, spFloat damping);

/// @}

#endif