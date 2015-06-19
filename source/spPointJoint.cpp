
#include "spPointJoint.h"

void 
spPointJointInit(spPointJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB)
{
    joint->constraint = spConstraintConstruct(a, b, SP_POINT_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->lambdaAccum = 0.0f;
    joint->eMass = 0.0f;
    joint->bias = 0.0f;
}

spPointJoint* 
spPointJointAlloc()
{
    return (spPointJoint*) spMalloc(sizeof(spPointJoint));
}

spPointJoint* 
spPointJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB)
{
    spPointJoint* joint = spPointJointAlloc();
    spPointJointInit(joint, a, b, anchorA, anchorB);
    return joint;
}

void 
spPointJointFree(spPointJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spPointJointPreSolve(spPointJoint* joint, const spFloat h)
{
}

void 
spPointJointSolve(spPointJoint* joint)
{
}