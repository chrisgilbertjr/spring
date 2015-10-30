
#include "spMotorJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define motorJoint spConstraintCastMotorJoint(constraint)

static void 
Free(spMotorJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spMotorJoint* joint, const spFloat h)
{
    /// compute the effective mass
    joint->eMass = 1.0f / (joint->constraint.bodyA->iInv + joint->constraint.bodyB->iInv);
}

static void 
WarmStart(spMotorJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    a->w -= joint->lambdaAccum * a->iInv;
    b->w += joint->lambdaAccum * b->iInv;

    /// clear lagrange multiplier
    joint->lambdaAccum = 0.0f;
}

static void 
Solve(spMotorJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the lagrange multiplier
    spFloat w = joint->w + b->w - a->w;
    spFloat lambda = joint->eMass * -w;
    spFloat lambdaPrev = joint->lambdaAccum;
    joint->lambdaAccum = lambdaPrev + lambda;

    /// compute the impulse
    spFloat impulse = joint->lambdaAccum - lambdaPrev;

    /// apply the impulse
    a->w -= impulse * a->iInv;
    b->w += impulse * b->iInv;
}

void 
spMotorJointInit(spMotorJoint* joint, spBody* a, spBody* b, spFloat w)
{
    joint->constraint = spConstraintConstruct(a, b, SP_MOTOR_JOINT);
    joint->lambdaAccum = 0.0f;
    joint->eMass = 0.0f;
    joint->w = w;
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
}

spMotorJoint* 
spMotorJointAlloc()
{
    return (spMotorJoint*) spMalloc(sizeof(spMotorJoint));
}

spConstraint* 
spMotorJointNew(spBody* a, spBody* b, spFloat w)
{
    spMotorJoint* joint = spMotorJointAlloc();
    NULLCHECK(joint);
    spMotorJointInit(joint, a, b, w);
    return (spConstraint*) joint;
}

spBool 
spConstraintIsMotorJoint(spConstraint* constraint)
{
    return constraint->type == SP_MOTOR_JOINT;
}

spMotorJoint* 
spConstraintCastMotorJoint(spConstraint* constraint)
{
    if (spConstraintIsMotorJoint(constraint))
    {
        return (spMotorJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a motor joint\n");
        return NULL;
    }
}

spFloat 
spMotorJointGetImpulse(spConstraint* constraint)
{
    return motorJoint->lambdaAccum;
}

spFloat 
spMotorJointGetAngVelocity(spConstraint* constraint)
{
    return motorJoint->w;
}

void 
spMotorJointSetAngVelocity(spConstraint* constraint, spFloat w)
{
    motorJoint->w = w;
}