
#ifndef SP_MOTOR_JOINT_H
#define SP_MOTOR_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spMotorJoint
{
    spConstraint constraint;
    spFloat lambdaAccum;
    spFloat inertia;
    spFloat w;
};

void spMotorJointInit(spMotorJoint* joint, spBody* a, spBody* b, spFloat w);

spMotorJoint* spMotorJointAlloc();

spMotorJoint* spMotorJointNew(spBody* a, spBody* b, spFloat w);

void spMotorJointFree(spMotorJoint** joint);

void spMotorJointPreStep(spMotorJoint* joint, const spFloat h);

void spMotorJointSolve(spMotorJoint* joint);

#endif