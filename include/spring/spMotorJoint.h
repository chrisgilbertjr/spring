
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
SPRING_API void spMotorJointInit(spMotorJoint* joint, spBody* a, spBody* b, spFloat w);

/// allocate a motor joint on the heap
SPRING_API spMotorJoint* spMotorJointAlloc();

/// init the motor joint with 2 bodies, and an angular velocity
SPRING_API spConstraint* spMotorJointNew(spBody* a, spBody* b, spFloat w);

/// check if a constraint is a motor joint
SPRING_API spBool spConstraintIsMotorJoint(spConstraint* constraint);

/// safely cast a constraint to a motor joint if its that type
SPRING_API spMotorJoint* spConstraintCastMotorJoint(spConstraint* constraint);

/// get the motor joints impulse
SPRING_API spFloat spMotorJointGetImpulse(spConstraint* constraint);

/// get the motor joints angular velocity
SPRING_API spFloat spMotorJointGetAngVelocity(spConstraint* constraint);

/// set the motor joints angular velocity
SPRING_API void spMotorJointSetAngVelocity(spConstraint* constraint, spFloat w);

/// @}

#endif