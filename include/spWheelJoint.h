
#ifndef SP_WHEEL_JOINT_H
#define SP_WHEEL_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spWheelJoint
{
    spConstraint constraint;
    spVector anchorA;
    spVector anchorB;
    spVector nLocal;
    spVector tLocal;
    spVector nWorld;
    spVector tWorld;
    spFloat sAx, sBx, sAy, sBy;
    spFloat lambdaAccumSpring;
    spFloat lambdaAccumMotor;
    spFloat lambdaAccumLine;
    spFloat maxMotorTorque;
    spFloat motorSpeed;
    spFloat eMassSpring;
    spFloat eMassMotor;
    spFloat eMassLine;
    spFloat frequency;
    spFloat damping;
    spFloat gamma;
    spFloat beta;
    spBool enableMotor;
};

void spWheelJointInit(spWheelJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping);

spWheelJoint* spWheelJointAlloc();

spWheelJoint* spWheelJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping);

void spWheelJointFree(spWheelJoint** joint);

void spWheelJointPreSolve(spWheelJoint* joint, const spFloat h);

void spWheelJointSolve(spWheelJoint* joint);

#endif