
#ifndef SP_MOTOR_JOINT_H
#define SP_MOTOR_JOINT_H

#include "spConstraint.h"

/// @defgroup spMotorJoint spMotorJoint
/// @{

struct spMotorJoint
{
    spConstraint constraint; ///< base constraint class
    spFloat lambdaAccum;     ///< lagrange multiple
    spFloat eMass;           ///< effective mass
    spFloat w;               ///< angular velocity
};

/// init the motor joint with 2 bodies, and an angular velocity
void spMotorJointInit(spMotorJoint* joint, spBody* a, spBody* b, spFloat w);

/// allocate a motor joint on the heap
spMotorJoint* spMotorJointAlloc();

/// init the motor joint with 2 bodies, and an angular velocity
spConstraint* spMotorJointNew(spBody* a, spBody* b, spFloat w);

/// check if a constraint is a motor joint
spBool spConstraintIsMotorJoint(spConstraint* constraint);

/// safely cast a constraint to a motor joint if its that type
spMotorJoint* spConstraintCastMotorJoint(spConstraint* constraint);

/// get the motor joints impulse
spFloat spMotorJointGetImpulse(spConstraint* constraint);

/// get the motor joints angular velocity
spFloat spMotorJointGetAngVelocity(spConstraint* constraint);

/// set the motor joints angular velocity
void spMotorJointSetAngVelocity(spConstraint* constraint, spFloat w);

/// @}

#endif