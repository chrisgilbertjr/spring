
#ifndef SP_HINGE_CONSTRAINT_H
#define SP_HINGE_CONSTRAINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spHingeJoint
{
    spConstraint base_class;
    spVector lanchor_a;
    spVector lanchor_b;
    spVector r_a;
    spVector r_b;
    spVector n;
    spFloat n_mass;
    spFloat jn_acc;
    spFloat bias;
};

void spHingeJointInit(spHingeJoint* constraint, spBody* a, spBody* b, spVector anchor_a, spVector anchor_b);

spHingeJoint* spHingeAlloc();

spHingeJoint* spHingeNew();

void spHingeJointFree(spHingeJoint* constraint);


#endif