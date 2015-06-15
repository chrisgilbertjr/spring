
#ifndef SP_SPRING_JOINT_H
#define SP_SPRING_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spSpringJoint
{
    spConstraint constraint;
    spVector anchorA;
    spVector anchorB;
    spVector rA;
    spVector rB;
    spVector n;
    spFloat restLength;
    spFloat frequency;
    spFloat damping;
    spFloat lambdaAccum;
    spFloat eMass;
    spFloat gamma;
    spFloat beta;
};

void spSpringJointInit(spSpringJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat restLength, spFloat frequency, spFloat damping);

spSpringJoint* spSpringJointAlloc();

spSpringJoint* spSpringJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat restLength, spFloat frequency, spFloat damping);

void spSpringJointFree(spSpringJoint** joint);

void spSpringJointPreStep(spSpringJoint* joint, const spFloat h);

void spSpringJointSolve(spSpringJoint* joint);

#endif