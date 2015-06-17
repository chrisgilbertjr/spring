
#ifndef SP_ANGULAR_SPRING_JOINT_H
#define SP_ANGULAR_SPRING_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spAngularSpringJoint
{
    spConstraint constraint;
    spFloat lambdaAccum;
    spFloat restAngle;
    spFloat frequency;
    spFloat damping;
    spFloat eMass;
    spFloat gamma;
    spFloat beta;
    spBool inverse;
};

void spAngularSpringJointInit(spAngularSpringJoint* joint, spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle);

spAngularSpringJoint* spAngularSpringJointAlloc();

spAngularSpringJoint* spAngularSpringJointNew(spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle);

void spAngularSpringJointFree(spAngularSpringJoint** joint);

void spAngularSpringJointPreSolve(spAngularSpringJoint* joint, const spFloat h);

void spAngularSpringJointSolve(spAngularSpringJoint* joint);

#endif