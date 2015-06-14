
#ifndef SP_DISTANCE_CONSTRAINT_H
#define SP_DISTANCE_CONSTRAINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spDistanceJoint
{
    spConstraint base_class;
    spVector anchor_a;
    spVector anchor_b;
    spVector rA;
    spVector rB;
    spVector n;
    spFloat distance;
    spFloat eMass;
    spFloat jAccum;
};

void spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance);

spDistanceJoint* spDistanceConstraintAlloc();

spDistanceJoint* spDistanceConstraintNew(spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance);

void spDistanceJointFree(spDistanceJoint** Joint);

void spDistanceJointPreStep(spDistanceJoint* joint, const spFloat h);

void spDistanceJointSolve(spDistanceJoint* joint);

void spDistanceJointStabilize(spDistanceJoint* joint);

spFloat spDistanceJointInvEffectiveMass(spFloat ima, spFloat imb, spFloat iia, spFloat iib, spVector ra, spVector rb, spVector n);

#ifdef SP_DEBUG
 #define spDistanceJointIsSane(distance_joint) __spDistanceJointIsSane(distance_joint)

 inline void __spDistanceJointIsSane(spDistanceJoint* joint)
 {
     /// TODO:
 }
#else
 #define spDistanceJointIsSane(distance_joint)
#endif

#endif