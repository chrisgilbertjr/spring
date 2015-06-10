
#ifndef SP_ANGLE_JOINT_H
#define SP_ANGLE_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

struct spAngleJoint
{
    spConstraint base_class;
    spFloat angle;
};

#endif