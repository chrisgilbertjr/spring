
#include "spDistanceJoint.h"

void 
spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance)
{
    joint->base_class = spConstraintConstruct(a, b, SP_DISTANCE_CONSTRAINT);
    joint->lanchor_a = anchor_a;
    joint->lanchor_b = anchor_b;
    joint->distance = distance;
}

spDistanceJoint* 
spDistanceConstraintAlloc()
{
    return (spDistanceJoint*) spMalloc(sizeof(spDistanceJoint));
}

spDistanceJoint* 
spDistanceConstraintNew(spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance)
{
    spDistanceJoint* joint = spDistanceConstraintAlloc();
    spDistanceJointInit(joint, a, b, anchor_a, anchor_b, distance);
    return joint;
}

void 
spDistanceJointFree(spDistanceJoint** joint)
{
    spFree(joint);
}

void 
spDistanceJointPreStep(spDistanceJoint* joint, const spFloat h)
{
    /// TODO;
}

void 
spDistanceJointSolve(spDistanceJoint* joint)
{
    /// TODO;
}

void 
spDistanceJointStabilize(spDistanceJoint* joint)
{
    /// TODO;
}