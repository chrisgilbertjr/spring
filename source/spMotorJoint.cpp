
#include "spMotorJoint.h"
#include "spBody.h"

void 
spMotorJointInit(spMotorJoint* joint, spBody* a, spBody* b, spFloat w)
{
    joint->constraint = spConstraintConstruct(a, b, SP_MOTOR_JOINT);
    joint->lambdaAccum = 0.0f;
    joint->inertia = 0.0f;
    joint->w = w;
}

spMotorJoint* 
spMotorJointAlloc()
{
    return (spMotorJoint*) spMalloc(sizeof(spMotorJoint));
}

spMotorJoint* 
spMotorJointNew(spBody* a, spBody* b, spFloat w)
{
    spMotorJoint* joint = spMotorJointAlloc();
    spMotorJointInit(joint, a, b, w);
    return joint;
}

void 
spMotorJointFree(spMotorJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spMotorJointPreStep(spMotorJoint* joint, const spFloat h)
{
    joint->inertia = 1.0f / (joint->constraint.bodyA->iInv + joint->constraint.bodyB->iInv);
}

void 
spMotorJointSolve(spMotorJoint* joint)
{
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    spFloat w = joint->w + b->w - a->w;
    spFloat lambda = joint->inertia * -w;
    spFloat lambdaPrev = joint->lambdaAccum;
    joint->lambdaAccum = lambdaPrev + lambda;
    spFloat impulse = joint->lambdaAccum - lambdaPrev;

    a->w -= impulse * a->iInv;
    b->w += impulse * b->iInv;
}