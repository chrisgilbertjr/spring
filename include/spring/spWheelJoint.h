
#ifndef SP_WHEEL_JOINT_H
#define SP_WHEEL_JOINT_H

#include "spConstraint.h"

/// @defgroup spWheelJoint spWheelJoint
/// @{

/// a wheel joint is a combination of a spring joint, point/line joint, and a motor joint
/// it is mainly used to create vehicles
struct spWheelJoint
{
    spConstraint constraint;    ///< base constraint class
    spVector anchorA;           ///< first anchor in body A's local space 
    spVector anchorB;           ///< second anchor in body B's local space
    spVector nLocal;            ///< temp solver variable
    spVector tLocal;            ///< temp solver variable
    spVector nWorld;            ///< temp solver variable
    spVector tWorld;            ///< temp solver variable
    spFloat sAx, sBx, sAy, sBy; ///< temp solver variables
    spFloat lambdaAccumSpring;  ///< spring lagrange multiplier
    spFloat lambdaAccumMotor;   ///< motor lagrange multiplier
    spFloat lambdaAccumLine;    ///< line lagrange multiplier
    spFloat maxMotorTorque;     ///< max motor torque
    spFloat motorSpeed;         ///< motor speed
    spFloat eMassSpring;        ///< spring effective mass
    spFloat eMassMotor;         ///< motor effective mass
    spFloat eMassLine;          ///< line effective mass
    spFloat frequency;          ///< spring frequency
    spFloat damping;            ///< spring damping
    spFloat gamma;              ///< spring solver variable
    spFloat beta;               ///< spring solver variable
    spFloat biasLine;           ///< spring solver variable
    spBool enableMotor;         ///< enable motor flag
};

/// init a wheel joint with 2 bodies, 2 local space anchors, a line axis, and spring frequency/damping
void spWheelJointInit(spWheelJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping);

/// allocate a wheel joint on the heap
SPRING_API spWheelJoint* spWheelJointAlloc();

/// create a wheel joint on the heap with 2 bodies, 2 local space anchors, a line axis, and spring frequency/damping
SPRING_API spConstraint* spWheelJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping);

/// check if a constraint is a wheel joint
SPRING_API spBool spConstraintIsWheelJoint(spConstraint* constraint);

/// safely cast a constraint to a wheel joint if its that type
SPRING_API spWheelJoint* spConstraintCastWheelJoint(spConstraint* constraint);

/// get the wheels impulse
SPRING_API spFloat spWheelJointGetImpulse(spConstraint* constraint);

/// get the wheels spring impulse
SPRING_API spFloat spWheelJointGetSpringImpulse(spConstraint* constraint);

/// get the wheels motor impulse
SPRING_API spFloat spWheelJointGetMotorImpulse(spConstraint* constraint);

/// get the wheels line impulse
SPRING_API spFloat spWheelJointGetLineImpulse(spConstraint* constraint);

/// get the first anchor in body A's local space
SPRING_API spVector spWheelJointGetAnchorA(spConstraint* constraint);

/// get the second anchor in body B's local space
SPRING_API spVector spWheelJointGetAnchorB(spConstraint* constraint);

/// get the first anchor in world space
SPRING_API spVector spWheelJointGetWorldAnchorA(spConstraint* constraint);

/// get the second anchor in world space
SPRING_API spVector spWheelJointGetWorldAnchorB(spConstraint* constraint);

/// get the motors last impulse
SPRING_API spFloat spWheelJointGetMotorImpulse(spConstraint* constraint);

/// get the wheel joints max motor torque
SPRING_API spFloat spWheelJointGetMaxMotorTorque(spConstraint* constraint);

/// get the wheel joints motor speed
SPRING_API spFloat spWheelJointGetMotorSpeed(spConstraint* constraint);

/// get the wheel joints spring frequency
SPRING_API spFloat spWheelJointGetSpringFrequency(spConstraint* constraint);

/// get the wheel joints spring damping
SPRING_API spFloat spWheelJointGetSpringDamping(spConstraint* constraint);

/// get if the wheel joints motor is enabled
SPRING_API spBool spWheelJointGetEnableMotor(spConstraint* constraint);

/// set the wheel joints first anchor in body A's local space
SPRING_API void spWheelJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the wheel joints second anchor in body B's local space
SPRING_API void spWheelJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the wheel joints first anchor in world space
SPRING_API void spWheelJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set the wheel joints second anchor in world space
SPRING_API void spWheelJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// set the wheel joints max motor torque
SPRING_API void spWheelJointSetMaxMotorTorque(spConstraint* constraint, spFloat maxTorque);

/// set the wheel joints speed
SPRING_API void spWheelJointSetMotorSpeed(spConstraint* constraint, spFloat speed);

/// set the wheel joints spring frequency
SPRING_API void spWheelJointSetSpringFrequency(spConstraint* constraint, spFloat frequency);

/// set the wheel joints spring damping
SPRING_API void spWheelJointSetSpringDamping(spConstraint* constraint, spFloat damping);

/// set if the wheel joints motor is enabled
SPRING_API void spWheelJointSetEnableMotor(spConstraint* constraint, spBool enabled);

/// @{

#endif