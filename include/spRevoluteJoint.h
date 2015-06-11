
#ifndef SP_REVOLUTE_JOINT_H
#define SP_REVOLUTE_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spRevoluteJoint
{
    spConstraint base_class;
    spFloat angle;
};

#endif