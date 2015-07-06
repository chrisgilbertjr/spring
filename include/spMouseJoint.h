
#ifndef SP_MOUSE_JOINT_H
#define SP_MOUSE_JOINT_H

#include "spConstraint.h"
#include "spBody.h"
#include "spMath.h"

struct spMouseJoint
{
    spConstraint constraint;
    spVector lambdaAccum;
    spVector anchor;
    spVector target;
    spVector bias;
    spVector rA;
    spFloat frequency;
    spFloat damping;
    spFloat gamma;
    spMatrix eMass;
};

void spMouseJointInit(spMouseJoint* joint, spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target);

spMouseJoint* spMouseJointAlloc();

spMouseJoint* spMouseJointNew(spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target);

void spMouseJointFree();

void spMouseJointPreSolve(spMouseJoint* joint, const spFloat h);

void spMouseJointSolve(spMouseJoint* joint);

inline void spMouseJointUpdate(spMouseJoint* joint, spVector target) 
{ 
    joint->target = target; 
}

inline void spMouseJointStart(spMouseJoint* joint, spBody* a, spVector point) 
{ 
    joint->constraint.bodyA = a; 
    joint->anchor = spTMult(a->xf, point);
    joint->target = point; 
}

inline void spMouseJointEnd(spMouseJoint* joint)
{
    joint->constraint.bodyA = NULL;
    joint->anchor = spVectorZero();
    joint->target = spVectorZero();
}

#endif