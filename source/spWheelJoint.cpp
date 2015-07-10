
#include "spWheelJoint.h" 
#include "spBody.h"
#include "spDebugDraw.h"

/// convenience macro for getters/setters
#define wheelJoint spConstraintCastWheelJoint(constraint)


static void 
Free(spWheelJoint** joint)
{
    NULLCHECK(joint);
    spFree(joint);
}

static void 
PreSolve(spWheelJoint* joint, const spFloat h)
{
    /// get the body
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute world anchors
    spVector pA = spMult(a->xf, joint->anchorA);
    spVector pB = spMult(b->xf, joint->anchorB);

    /// compute rel velocity
    spVector rA = spSub(pA, a->p);
    spVector rB = spSub(pB, b->p);
    spVector dir = spSub(pB, pA);

    /// pre solve the point/line constraint
    {
        /// compute effective mass
        joint->tWorld = spMult(a->xf.q, joint->tLocal);
        joint->sAy = spCross(spAdd(dir, rA), joint->tWorld);
        joint->sBy = spCross(rB, joint->tWorld);

        spFloat rvA = a->mInv + a->iInv * joint->sAy * joint->sAy;
        spFloat rvB = b->mInv + b->iInv * joint->sBy * joint->sBy;

        joint->eMassLine = rvA + rvB;
        joint->eMassLine = joint->eMassLine ? 1.0f / joint->eMassLine : 0.0f;
    }

    /// pre solve the soft spring constraint
    joint->eMassSpring = 0.0f;
    joint->gamma = 0.0f;
    joint->beta = 0.0f;
    if (joint->frequency > 0.0f)
    {
        /// compute inverse mass and effective mass
        joint->nWorld = spMult(a->xf.q, joint->nLocal);
        joint->sAx = spCross(spAdd(dir, rA), joint->nWorld);
        joint->sBx = spCross(rB, joint->nWorld);

        spFloat iMassA = a->mInv + a->iInv * joint->sAx * joint->sAx;
        spFloat iMassB = b->mInv + b->iInv * joint->sBx * joint->sBx;
        spFloat iMass = iMassA + iMassB;

        if (iMass > 0.0f)
        {
            /// solve spring params
            joint->eMassSpring = 1.0f / iMass;
            spFloat C = spDot(dir, joint->nWorld);
            spFloat omega = 2.0f * SP_PI * joint->frequency;
            spFloat c = 2.0f * joint->eMassSpring * joint->damping * omega;
            spFloat k = joint->eMassSpring * omega * omega;
            joint->gamma = h * (h * k + c);
            joint->gamma = joint->gamma ? 1.0f / joint->gamma : 0.0f;
            joint->beta = joint->gamma * C * h * k;

            /// compute effective mass
            joint->eMassSpring += joint->gamma;
            joint->eMassSpring  = joint->eMassSpring ? 1.0f / joint->eMassSpring : 0.0f;
        }
    }

    /// pre solve the motor constraint
    if (joint->enableMotor)
    {
        /// compute effective mass
        joint->eMassMotor = a->iInv + b->iInv;
        joint->eMassMotor = joint->eMassMotor ? 1.0f / joint->eMassMotor : 0.0f;
    }
    else
    {
        joint->eMassMotor = 0.0f;
    }
}

static void 
WarmStart(spWheelJoint* joint)
{
    /// get the body
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// spring impulses
    {
        /// compute the impulse
        spVector impulse = spMult(joint->nWorld, joint->lambdaAccumSpring);
        spFloat impulseA = joint->lambdaAccumSpring * joint->sAx;
        spFloat impulseB = joint->lambdaAccumSpring * joint->sBx;

        /// apply the impulse
        a->v = spSub(a->v, spMult(a->mInv, impulse));
        b->v = spAdd(b->v, spMult(b->mInv, impulse));
        a->w -= a->iInv * impulseA;
        b->w += b->iInv * impulseB;
    }

    /// motor impulses
    {
        /// apply the impulse
        a->w -= a->iInv * joint->lambdaAccumMotor;
        b->w += b->iInv * joint->lambdaAccumMotor;
    }

    /// point/line impulses
    {
        /// compute the impulses
        spVector impulse = spMult(joint->lambdaAccumLine, joint->tWorld);
        spFloat impulseA = joint->sAy * joint->lambdaAccumLine;
        spFloat impulseB = joint->sBy * joint->lambdaAccumLine;

        /// apply the impulses
        a->v = spSub(a->v, spMult(a->mInv, impulse));
        b->v = spAdd(b->v, spMult(b->mInv, impulse));
        a->w -= a->iInv * impulseA;
        b->w += b->iInv * impulseB;
    }

    /// reset lagrange multipliers
    joint->lambdaAccumSpring = 0.0f;
    joint->lambdaAccumMotor = 0.0f;
    joint->lambdaAccumLine = 0.0f;
}

static void 
Solve(spWheelJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// solve spring constraint
    {
        /// compute the velocity constraint and compute the multiplier
        spFloat Cdot = spDot(joint->nWorld, spSub(b->v, a->v)) + joint->sBx * b->w - joint->sAx * a->w;
        spFloat lambda = -joint->eMassSpring * (Cdot + joint->beta + joint->gamma * joint->lambdaAccumSpring);
        joint->lambdaAccumSpring += lambda;

        /// compute the impulses
        spVector impulse = spMult(joint->nWorld, lambda);
        spFloat impulseA = lambda * joint->sAx;
        spFloat impulseB = lambda * joint->sBx;

        /// apply the impulses
        a->v = spSub(a->v, spMult(a->mInv, impulse));
        b->v = spAdd(b->v, spMult(b->mInv, impulse));
        a->w -= a->iInv * impulseA;
        b->w += b->iInv * impulseB;
    }

    /// solve motor constraint
    {
        /// compute the velocity constraint and compute the multiplier
        spFloat Cdot = b->w - a->w + joint->motorSpeed;
        spFloat lambda = -Cdot * joint->motorSpeed;
        spFloat lambdaMax = joint->maxMotorTorque * 1.0f / 60.0f;
        spFloat lambdaOld = joint->lambdaAccumMotor;
        joint->lambdaAccumMotor = spClamp(joint->lambdaAccumMotor + lambda, -lambdaMax, lambdaMax);

        /// compute the impulse
        spFloat impulse = joint->lambdaAccumMotor - lambdaOld;

        /// apply the impulse
        a->w -= a->iInv * impulse;
        b->w += b->iInv * impulse;
    }

    /// solve point/line constraint
    {
        /// compute the velocity constraint and compute the multiplier
        spFloat Cdot = spDot(joint->tWorld, spSub(b->v, a->v)) + joint->sBy * b->w - joint->sAy * a->w;
        spFloat lambda = -joint->eMassLine * Cdot;
        joint->lambdaAccumLine += lambda;

        /// compute the impulses
        spVector impulse = spMult(lambda, joint->tWorld);
        spFloat impulseA = joint->sAy * lambda;
        spFloat impulseB = joint->sBy * lambda;

        /// apply the impulses
        a->v = spSub(a->v, spMult(a->mInv, impulse));
        b->v = spAdd(b->v, spMult(b->mInv, impulse));
        a->w -= a->iInv * impulseA;
        b->w += b->iInv * impulseB;
    }
}


void 
spWheelJointInit(spWheelJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping)
{
    joint->constraint = spConstraintConstruct(a, b, SP_WHEEL_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->nLocal = axis;
    joint->tLocal = spSkewT(axis);
    joint->tLocal = spCross(1.0f, axis);
    joint->nWorld = spVectorZero();
    joint->tWorld = spVectorZero();
    joint->sAx = joint->sBx = joint->sAy = joint->sBy = 0.0f;
    joint->lambdaAccumSpring = 0.0f;
    joint->lambdaAccumMotor = 0.0f;
    joint->lambdaAccumLine = 0.0f;
    joint->maxMotorTorque = 0.0f;
    joint->motorSpeed = 0.0f;
    joint->eMassSpring = 0.0f;
    joint->eMassMotor = 0.0f;
    joint->eMassLine = 0.0f;
    joint->frequency = frequency;
    joint->damping = damping;
    joint->gamma = 0.0f;
    joint->beta = 0.0f;
    joint->enableMotor = spFalse;
}

spWheelJoint* 
spWheelJointAlloc()
{
    return (spWheelJoint*) spMalloc(sizeof(spWheelJoint));
}

spConstraint* 
spWheelJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping)
{
    spWheelJoint* joint = spWheelJointAlloc();
    NULLCHECK(joint);
    spWheelJointInit(joint, a, b, anchorA, anchorB, axis, frequency, damping);
    return (spConstraint*) joint;
}
spBool 
spConstraintIsWheelJoint(spConstraint* constraint)
{
    return constraint->type == SP_WHEEL_JOINT;
}

spWheelJoint* 
spConstraintCastWheelJoint(spConstraint* constraint)
{
    if (spConstraintIsWheelJoint(constraint))
    {
        return (spWheelJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a wheel joint\n");
        return NULL;
    }
}

spFloat 
spWheelJointGetImpulse(spConstraint* constraint)
{
    return wheelJoint->lambdaAccumLine  +
           wheelJoint->lambdaAccumMotor +
           wheelJoint->lambdaAccumSpring;
}

spFloat 
spWheelJointGetSpringImpulse(spConstraint* constraint)
{
    return wheelJoint->lambdaAccumSpring;
}

spFloat 
spWheelJointGetMotorImpulse(spConstraint* constraint)
{
    return wheelJoint->lambdaAccumMotor;
}

spFloat 
spWheelJointGetLineImpulse(spConstraint* constraint)
{
    return wheelJoint->lambdaAccumLine;
}

spVector 
spWheelJointGetAnchorA(spConstraint* constraint)
{
    return wheelJoint->anchorA;
}

spVector 
spWheelJointGetAnchorB(spConstraint* constraint)
{
    return wheelJoint->anchorB;
}

spVector 
spWheelJointGetWorldAnchorA(spConstraint* constraint)
{
    return spMult(constraint->bodyA->xf, wheelJoint->anchorA);
}

spVector 
spWheelJointGetWorldAnchorB(spConstraint* constraint)
{
    return spMult(constraint->bodyB->xf, wheelJoint->anchorB);
}

spFloat 
spWheelJointGetMaxMotorTorque(spConstraint* constraint)
{
    return wheelJoint->maxMotorTorque;
}

spFloat 
spWheelJointGetMotorSpeed(spConstraint* constraint)
{
    return wheelJoint->motorSpeed;
}

spFloat 
spWheelJointGetSpringFrequency(spConstraint* constraint)
{
    return wheelJoint->frequency;
}

spFloat 
spWheelJointGetSpringDamping(spConstraint* constraint)
{
    return wheelJoint->damping;
}

spBool 
spWheelJointGetEnableMotor(spConstraint* constraint)
{
    return wheelJoint->enableMotor;
}

void 
spWheelJointSetAnchorA(spConstraint* constraint, spVector anchorA)
{
    wheelJoint->anchorA = anchorA;
}

void 
spWheelJointSetAnchorB(spConstraint* constraint, spVector anchorB)
{
    wheelJoint->anchorB = anchorB;
}

void 
spWheelJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA)
{
    wheelJoint->anchorA = spMult(constraint->bodyA->xf, anchorA);
}

void 
spWheelJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    wheelJoint->anchorB = spMult(constraint->bodyB->xf, anchorB);
}

void 
spWheelJointSetMaxMotorTorque(spConstraint* constraint, spFloat maxTorque)
{
    wheelJoint->maxMotorTorque = maxTorque;
}

void 
spWheelJointSetSpringFrequency(spConstraint* constraint, spFloat frequency)
{
    wheelJoint->frequency = frequency;
}

void 
spWheelJointSetSpringDamping(spConstraint* constraint, spFloat damping)
{
    wheelJoint->damping = damping;
}

void 
spWheelJointSetEnableMotor(spConstraint* constraint, spBool enabled)
{
    wheelJoint->enableMotor = enabled;
}