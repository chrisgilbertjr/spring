
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

/// free a mouse joint from the heap
void spMouseJointFree();

/// setup a mouse joint to be solved
void spMouseJointPreSolve(spMouseJoint* joint, const spFloat h);

/// solve the mouse joint, and apply impulses to the body
void spMouseJointSolve(spMouseJoint* joint);

/// start a mouse drag on body a given a world anchor point
void spMouseJointStart(spMouseJoint* joint, spBody* a, spVector point);

/// end a mouse drag 
void spMouseJointEnd(spMouseJoint* joint);

/// get the impulse used on both bodies (this is not the applied impulse)
spVector spMouseJointGetImpulse(spMouseJoint* joint);

/// get the body anchor in local space
spVector spMouseJointGetAnchor(spMouseJoint* joint);

/// get the body anchor in world space
spVector spMouseJointGetWorldAnchor(spMouseJoint* joint);

/// get the target in world space
spVector spMouseJointGetTarget(spMouseJoint* joint);

/// get the target in body A's local space
spVector spMouseJointGetLocalTarget(spMouseJoint* joint);

/// get the spring frequency of the mouse joint
spFloat spMouseJointGetFrequency(spMouseJoint* joint);

/// get the spring damping of the mouse joint
spFloat spMouseJointGetDamping(spMouseJoint* joint);

/// set the mouse joints anchor in body A's local space
void spMouseJointSetAnchor(spMouseJoint* joint, spVector anchor);

/// set the mouse joints anchor in world space
void spMouseJointSetWorldAnchor(spMouseJoint* joint, spVector anchor);

/// set the mouse joints target with a world space point
void spMouseJointSetTarget(spMouseJoint* joint, spVector target);

/// set the mouse joints target with a point in body A's local space
void spMouseJointSetLocalTarget(spMouseJoint* joint, spVector target);

/// set the spring frequency of the mouse joint
void spMouseJointSetFrequency(spMouseJoint* joint, spFloat frequency);

/// set the spring damping of the mouse joint
void spMouseJointSetDamping(spMouseJoint* joint, spFloat damping);

/// @}

#endif