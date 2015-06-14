
#ifndef SP_ROPE_JOINT_H
#define SP_ROPE_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spRopeJoint
{
    spConstraint constraint;
    spVector anchorA;
    spVector anchorB;
    spVector rA;
    spVector rB;
    spVector n;
    spFloat maxDistance;
    spFloat eMass;
    spFloat jAccum;
    spFloat bias;
    spBool shorterThanMax;
};

void spRopeJointInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance);

spRopeJoint* spRopeJointAlloc();

spRopeJoint* spRopeJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance);

void spRopeJointFree(spRopeJoint** joint);

void spRopeJointPreStep(spRopeJoint* joint, const spFloat h);

void spRopeJointSolve(spRopeJoint* joint);

void spRopeJointStabilize(spRopeJoint* joint);

#ifdef SP_DEBUG
 #define spRopeJointIsSane(distance_joint) __spRopeJointIsSane(distance_joint)

 inline void __spRopeJointIsSane(spRopeJoint* joint)
 {
     /// TODO:
 }
#else
 #define spRopeJointIsSane(distance_joint)
#endif

#endif