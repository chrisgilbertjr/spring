
#ifndef SP_POINT_JOINT_H
#define SP_POINT_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spPointJoint
{
    spConstraint constraint;
    spVector anchorA;
    spVector anchorB;
    spVector lambdaAccum;
    spVector bias;
    spVector rA;
    spVector rB;
    spMatrix eMass;
};

void spPointJointInit(spPointJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB);

spPointJoint* spPointJointAlloc();

spPointJoint* spPointJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB);

void spPointJointFree(spPointJoint** joint);

void spPointJointPreSolve(spPointJoint* joint, const spFloat h);

void spPointJointSolve(spPointJoint* joint);

#endif