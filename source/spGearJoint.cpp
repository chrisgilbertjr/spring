
#include "spGearJoint.h"
#include "spBody.h"

void 
spGearJointInit(spGearJoint* joint, spBody* a, spBody* b, spFloat ratio, spFloat phase)
{
    joint->constraint = spConstraintConstruct(a, b, SP_GEAR_JOINT);
    joint->lambdaAccum = 0.0f;
    joint->ratioInv = ratio ? 1.0f / ratio : 0.0f;
    joint->ratio = ratio;
    joint->eMass = 0.0f;
    joint->phase = phase;
    joint->bias = 0.0f;
}

spGearJoint* 
spGearJointAlloc()
{
    return (spGearJoint*) spMalloc(sizeof(spGearJoint));
}

spGearJoint* 
spGearJointNew(spBody* a, spBody* b, spFloat ratio, spFloat phase)
{
    spGearJoint* joint = spGearJointAlloc();
    spGearJointInit(joint, a, b, ratio, phase);
    return joint;
}

void 
spGearJointFree(spGearJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spGearJointPreSolve(spGearJoint* joint, const spFloat h)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    spFloat iMass = bA->i_inv*joint->ratioInv + bB->i_inv*joint->ratio;
    joint->eMass = iMass ? 1.0f / iMass : 0.0f;

    spFloat C = bA->a * joint->ratio - bB->a - joint->phase;
    spFloat beta = 0.3f;
    joint->bias =  C * beta / h;
}

void 
spGearJointSolve(spGearJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    spFloat Cdot = bB->w * joint->ratio - bA->w;
    spFloat lambda = (joint->bias - Cdot) * joint->eMass;
    spFloat lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum += lambda;
    spFloat impulse = joint->lambdaAccum - lambdaOld;;

    bA->w -= impulse * bA->i_inv * joint->ratioInv;
    bB->w += impulse * bB->i_inv;
}