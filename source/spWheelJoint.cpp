
#include "spWheelJoint.h" 
#include "spBody.h"
#include "spDebugDraw.h"

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

spWheelJoint* 
spWheelJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spVector axis, spFloat frequency, spFloat damping)
{
    spWheelJoint* joint = spWheelJointAlloc();
    spWheelJointInit(joint, a, b, anchorA, anchorB, axis, frequency, damping);
    return joint;
}

void 
spWheelJointFree(spWheelJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spWheelJointPreSolve(spWheelJoint* joint, const spFloat h)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    spVector pA = spMult(bA->xf, joint->anchorA);
    spVector pB = spMult(bB->xf, joint->anchorB);
    spVector rA = spSub(pA, bA->p);
    spVector rB = spSub(pB, bB->p);
    spVector dir = spSub(pB, pA);

    spDebugDrawLine(pA, pB, spRed(1.0f));

    /// pre solve the point/line constraint
    {
        joint->tWorld = spMult(bA->xf.q, joint->tLocal);
        joint->sAy = spCross(spAdd(dir, rA), joint->tWorld);
        joint->sBy = spCross(rB, joint->tWorld);

        spFloat rvA = bA->m_inv + bA->i_inv * joint->sAy * joint->sAy;
        spFloat rvB = bB->m_inv + bB->i_inv * joint->sBy * joint->sBy;
        joint->eMassLine = rvA + rvB;
        joint->eMassLine = joint->eMassLine ? 1.0f / joint->eMassLine : 0.0f;
    }

    /// pre solve the soft spring constraint
    joint->eMassSpring = 0.0f;
    joint->gamma = 0.0f;
    joint->beta = 0.0f;
    if (joint->frequency > 0.0f)
    {
        joint->nWorld = spMult(bA->xf.q, joint->nLocal);
        joint->sAx = spCross(spAdd(dir, rA), joint->nWorld);
        joint->sBx = spCross(rB, joint->nWorld);

        spFloat iMassA = bA->m_inv + bA->i_inv * joint->sAx * joint->sAx;
        spFloat iMassB = bB->m_inv + bB->i_inv * joint->sBx * joint->sBx;
        spFloat iMass = iMassA + iMassB;

        if (iMass > 0.0f)
        {
            joint->eMassSpring = 1.0f / iMass;
            spFloat C = spDot(dir, joint->nWorld);
            spFloat omega = 2.0f * SP_PI * joint->frequency;
            spFloat c = 2.0f * joint->eMassSpring * joint->damping * omega;
            spFloat k = joint->eMassSpring * omega * omega;
            joint->gamma = h * (h * k + c);
            joint->gamma = joint->gamma ? 1.0f / joint->gamma : 0.0f;
            joint->beta = joint->gamma * C * h * k;
            joint->eMassSpring += joint->gamma;
            joint->eMassSpring  = joint->eMassSpring ? 1.0f / joint->eMassSpring : 0.0f;
        }
    }

    /// pre solve the motor constraint
    if (joint->enableMotor)
    {
        joint->eMassMotor = bA->i_inv + bB->i_inv;
        joint->eMassMotor = joint->eMassMotor ? 1.0f / joint->eMassMotor : 0.0f;
    }
    else
    {
        joint->eMassMotor = 0.0f;
    }

    joint->lambdaAccumSpring = 0.0f;
    joint->lambdaAccumMotor = 0.0f;
    joint->lambdaAccumLine = 0.0f;
}

void 
spWheelJointSolve(spWheelJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    /// solve spring constraint
    {
        spFloat Cdot = spDot(joint->nWorld, spSub(bB->v, bA->v)) + joint->sBx * bB->w - joint->sAx * bA->w;
        spFloat lambda = -joint->eMassSpring * (Cdot + joint->beta + joint->gamma * joint->lambdaAccumSpring);
        joint->lambdaAccumSpring += lambda;

        spVector impulse = spMult(joint->nWorld, lambda);
        spFloat impulseA = lambda * joint->sAx;
        spFloat impulseB = lambda * joint->sBx;

        bA->v = spSub(bA->v, spMult(bA->m_inv, impulse));
        bB->v = spAdd(bB->v, spMult(bB->m_inv, impulse));
        bA->w -= bA->i_inv * impulseA;
        bB->w += bB->i_inv * impulseB;
    }

    /// solve motor constraint
    {
        spFloat Cdot = bB->w - bA->w + joint->motorSpeed;
        spFloat lambda = -Cdot * joint->motorSpeed;
        spFloat lambdaMax = joint->maxMotorTorque * 1.0f / 60.0f;
        spFloat lambdaOld = joint->lambdaAccumMotor;
        joint->lambdaAccumMotor = spClamp(joint->lambdaAccumMotor + lambda, -lambdaMax, lambdaMax);
        spFloat impulse = joint->lambdaAccumMotor - lambdaOld;

        bA->w -= bA->i_inv * impulse;
        bB->w += bB->i_inv * impulse;
    }

    /// solve point/line constraint
    {
        spFloat Cdot = spDot(joint->tWorld, spSub(bB->v, bA->v)) + joint->sBy * bB->w - joint->sAy * bA->w;
        spFloat lambda = -joint->eMassLine * Cdot;
        joint->lambdaAccumLine += lambda;

        spVector impulse = spMult(lambda, joint->tWorld);
        spFloat impulseA = joint->sAy * lambda;
        spFloat impulseB = joint->sBy * lambda;

        bA->v = spSub(bA->v, spMult(bA->m_inv, impulse));
        bB->v = spAdd(bB->v, spMult(bB->m_inv, impulse));
        bA->w -= bA->i_inv * impulseA;
        bB->w += bB->i_inv * impulseB;
    }
}