
#include "spDebugDraw.h"
#include "spMouseJoint.h"
#include "spBody.h"

void 
spMouseJointInit(spMouseJoint* joint, spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target)
{
    NULLCHECK(joint);
    joint->constraint = spConstraintConstruct(a, NULL, SP_MOUSE_JOINT);
    joint->lambdaAccum = spVectorZero();
    joint->anchor = anchor;
    joint->target = target;
    joint->bias = spVectorZero();
    joint->rA = spVectorZero();
    joint->frequency = frequency;
    joint->damping = damping;
    joint->gamma = 0.0f;
    joint->eMass = spMatrixZero();
}

spMouseJoint* 
spMouseJointAlloc()
{
    return (spMouseJoint*) spMalloc(sizeof(spMouseJoint));
}

spMouseJoint* 
spMouseJointNew(spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target)
{
    spMouseJoint* joint = spMouseJointAlloc();
    NULLCHECK(joint);
    spMouseJointInit(joint, a, frequency, damping, anchor, target);
    return joint;
}

void 
spMouseJointFree(spMouseJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

void 
spMouseJointPreSolve(spMouseJoint* joint, const spFloat h)
{
    /// get the body
    spBody* a = joint->constraint.bodyA;
    
    /// compute spring params
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * a->m * joint->damping * omega;
    spFloat k = a->m * omega * omega;

    spFloat hk = h * k;
    joint->gamma = h * (c + hk);
    joint->gamma = joint->gamma ? 1.0f / joint->gamma : 0.0f;
    spFloat beta = hk * joint->gamma;

    joint->rA = spMult(a->xf.q, spSub(joint->anchor, a->com));

    /// compute mass matrix
    spMatrix K = spMatrixZero();
    K.b = K.c = -a->iInv * joint->rA.x * joint->rA.y;
    K.a = a->mInv + a->iInv * joint->rA.y * joint->rA.y + joint->gamma;
    K.d = a->mInv + a->iInv * joint->rA.x * joint->rA.x + joint->gamma;

    /// compute effective mass
    joint->eMass = spInverse(K);

    a->w *= .98f;

    /// compute the position constraint and baumgarte velocity bias
    spVector C = spAdd(a->p, spSub(joint->rA, joint->target));
    joint->bias = spMult(C, beta);
    joint->lambdaAccum = spVectorZero();
}

void 
spMouseJointSolve(spMouseJoint* joint)
{
    /// get the body
    spBody* a = joint->constraint.bodyA;

    /// compute velocity constraint and compute the multiplier
    spVector Cdot = spAdd(a->v, spCross(a->w, joint->rA));
    spVector A = spAdd(Cdot, joint->bias);
    spVector B = spMult(joint->gamma, joint->lambdaAccum);
    spVector D = spNegate(spAdd(A, B));

    /// accumulate the impulse
    spVector lambdaOld = joint->lambdaAccum;
    spVector lambda = spMult(joint->eMass, D);
    joint->lambdaAccum = spAdd(joint->lambdaAccum, lambda);
    spVector impulse = spSub(joint->lambdaAccum, lambdaOld);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulse);
    //a->v  = spAdd(a->v, spMult(a->mInv, impulse));
    //a->w += a->iInv * spCross(joint->rA, impulse);
}

spBool 
spConstraintIsMouseJoint(spConstraint* constraint)
{
    return constraint->type == SP_MOUSE_JOINT;
}

spMouseJoint* 
spConstraintCastMouseJoint(spConstraint* constraint)
{
    if (spConstraintIsMouseJoint(constraint))
    {
        return (spMouseJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a mouse joint\n");
        return NULL;
    }
}

void 
spMouseJointStart(spMouseJoint* joint, spBody* a, spVector point)
{ 
    joint->constraint.bodyA = a; 
    joint->anchor = spTMult(a->xf, point);
    joint->target = point; 
}

void spMouseJointEnd(spMouseJoint* joint)
{
    joint->constraint.bodyA = NULL;
    joint->anchor = spVectorZero();
    joint->target = spVectorZero();
}

spVector 
spMouseJointGetImpulse(spMouseJoint* joint)
{
    return joint->lambdaAccum;
}

spVector 
spMouseJointGetAnchor(spMouseJoint* joint)
{
    return joint->anchor;
}

spVector 
spMouseJointGetWorldAnchor(spMouseJoint* joint)
{
    return spMult(joint->constraint.bodyA->xf, joint->anchor);
}

spVector 
spMouseJointGetTarget(spMouseJoint* joint)
{
    return joint->target;
}

spVector 
spMouseJointGetLocalTarget(spMouseJoint* joint)
{
    return spTMult(joint->constraint.bodyA->xf, joint->target);
}

spFloat 
spMouseJointGetFrequency(spMouseJoint* joint)
{
    return joint->frequency;
}

spFloat 
spMouseJointGetDamping(spMouseJoint* joint)
{
    return joint->damping;
}

void 
spMouseJointSetAnchor(spMouseJoint* joint, spVector anchor)
{
    joint->anchor = anchor;
}

void 
spMouseJointSetWorldAnchor(spMouseJoint* joint, spVector anchor)
{
    joint->anchor = spTMult(joint->constraint.bodyA->xf, anchor);
}

void 
spMouseJointSetTarget(spMouseJoint* joint, spVector target)
{
    joint->target = target;
}

void 
spMouseJointSetLocalTarget(spMouseJoint* joint, spVector target)
{
    joint->target = spMult(joint->constraint.bodyA->xf, target);
}

void 
spMouseJointSetFrequency(spMouseJoint* joint, spFloat frequency)
{
    joint->frequency = frequency;
}

void 
spMouseJointSetDamping(spMouseJoint* joint, spFloat damping)
{
    joint->damping = damping;
}