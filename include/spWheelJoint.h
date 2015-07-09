
#ifndef SP_WHEEL_JOINT_H
#define SP_WHEEL_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

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
    spBool enableMotor;         ///< enable motor flag
};

/// init a wheel joint with 2 bodies, 2 local space anchors, a line axis, and spring frequency/damping
void spWheelJointInit(spWheelJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping);

/// allocate a wheel joint on the heap
spWheelJoint* spWheelJointAlloc();

/// create a wheel joint on the heap with 2 bodies, 2 local space anchors, a line axis, and spring frequency/damping
spConstraint* spWheelJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping);

/// free a wheel joint on the heap
void spWheelJointFree(spWheelJoint** joint);

/// setup a wheel joint to be solved
void spWheelJointPreSolve(spWheelJoint* joint, const spFloat h);

/// warm start the joint from last timestep
void spWheelJointApplyCachedImpulse(spWheelJoint* joint);

/// solve the wheel joint and apply impulses
void spWheelJointSolve(spWheelJoint* joint);

/// check if a constraint is a wheel joint
spBool spConstraintIsWheelJoint(spConstraint* constraint);

/// safely cast a constraint to a wheel joint if its that type
spWheelJoint* spConstraintCastWheelJoint(spConstraint* constraint);

/// get the first anchor in body A's local space
spVector spWheelJointGetAnchorA(spConstraint* constraint);

/// get the second anchor in body B's local space
spVector spWheelJointGetAnchorB(spConstraint* constraint);

/// get the first anchor in world space
spVector spWheelJointGetWorldAnchorA(spConstraint* constraint);

/// get the second anchor in world space
spVector spWheelJointGetWorldAnchorB(spConstraint* constraint);

/// get the springs velocity impulse
spVector spWheelJointGetSpringImpulse(spConstraint* constraint);

/// get the springs angular impulse on body A
spFloat spWheelJointGetSpringImpulseA(spConstraint* constraint);

/// get the springs angular impulse on body B
spFloat spWheelJointGetSpringImpulseB(spConstraint* constraint);

/// get the motors last impulse
spFloat spWheelJointGetMotorImpulse(spConstraint* constraint);

/// get the lines last velocity impulse
spVector spWheelJointGetLineImpulse(spConstraint* constraint);

/// get the lines last angular impulse on body A
spFloat spWheelJointGetLineImpulseA(spConstraint* constraint);

/// get the lines last angular impulse on body B
spFloat spWheelJointGetLineImpulseB(spConstraint* constraint);

/// get the wheel joints max motor torque
spFloat spWheelJointGetMaxMotorTorque(spConstraint* constraint);

/// get the wheel joints motor speed
spFloat spWheelJointGetMotorSpeed(spConstraint* constraint);

/// get the wheel joints spring frequency
spFloat spWheelJointGetSpringFrequency(spConstraint* constraint);

/// get the wheel joints spring damping
spFloat spWheelJointGetSpringDamping(spConstraint* constraint);

/// get if the wheel joints motor is enabled
spBool spWheelJointGetEnableMotor(spConstraint* constraint);

/// set the wheel joints first anchor in body A's local space
void spWheelJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set the wheel joints second anchor in body B's local space
void spWheelJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set the wheel joints first anchor in world space
void spWheelJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set the wheel joints second anchor in world space
void spWheelJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// set the wheel joints max motor torque
void spWheelJointSetMaxMotorTorque(spConstraint* constraint, spFloat maxTorque);

/// set the wheel joints spring frequency
void spWheelJointSetSpringFrequency(spConstraint* constraint, spFloat frequency);

/// set the wheel joints spring damping
void spWheelJointSetSpringDamping(spConstraint* constraint, spFloat damping);

/// set if the wheel joints motor is enabled
void spWheelJointSetEnableMotor(spConstraint* constraint, spBool enabled);

/// @{

#endif