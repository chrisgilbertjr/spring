
#ifndef SP_GEAR_JOINT_H
#define SP_GEAR_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spGearJoint
{
    spConstraint constraint;
    spFloat lambdaAccum;
    spFloat ratioInv;
    spFloat ratio;
    spFloat eMass;
    spFloat phase;
    spFloat bias;
};

void spGearJointInit(spGearJoint* joint, spBody* a, spBody* b, spFloat ratio, spFloat phase);

spGearJoint* spGearJointAlloc();

spGearJoint* spGearJointNew(spBody* a, spBody* b, spFloat ratio, spFloat phase);

void spGearJointFree(spGearJoint** joint);

void spGearJointPreSolve(spGearJoint* joint, const spFloat h);

void spGearJointSolve(spGearJoint* joint);

#endif