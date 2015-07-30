
#include "spMouseJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define mouseJoint spConstraintCastMouseJoint(constraint)

static void 
Free(spMouseJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spMouseJoint* joint, const spFloat h)
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
    spVector C = spSub(spMult(a->xf, joint->anchor), joint->target);
    joint->bias = spMult(C, beta);
    joint->lambdaAccum = spVectorZero();
}

static void 
WarmStart(spMouseJoint** joint) {}

static void 
Solve(spMouseJoint* joint)
{
    /// get the body
    spBody* a = joint->constraint.bodyA;

    /// compute velocity constraint and compute the multiplier
    spVector Cdot = spAdd(a->v, spCross(a->w, joint->rA));
    spVector A = spAdd(Cdot, joint->bias);
    spVector B = spMult(joint->gamma, joint->lambdaAccum);
    spVector D = spNegative(spAdd(A, B));

    /// accumulate the impulse
    spVector lambda = spMult(joint->eMass, D);
    spVector lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum = spAdd(joint->lambdaAccum, lambda);
    spVector impulse = spSub(joint->lambdaAccum, lambdaOld);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulse);
}

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
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
}

spMouseJoint* 
spMouseJointAlloc()
{
    return (spMouseJoint*) spMalloc(sizeof(spMouseJoint));
}

spConstraint* 
spMouseJointNew(spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target)
{
    spMouseJoint* joint = spMouseJointAlloc();
    NULLCHECK(joint);
    spMouseJointInit(joint, a, frequency, damping, anchor, target);
    return (spConstraint*) joint;
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
spMouseJointStart(spConstraint* constraint, spBody* a, spVector point)
{ 
    mouseJoint->constraint.bodyA = a; 
    mouseJoint->anchor = spTMult(a->xf, point);
    mouseJoint->target = point; 
}

void spMouseJointEnd(spConstraint* constraint)
{
    mouseJoint->constraint.bodyA = NULL;
    mouseJoint->anchor = spVectorZero();
    mouseJoint->target = spVectorZero();
}

spVector 
spMouseJointGetImpulse(spConstraint* constraint)
{
    return mouseJoint->lambdaAccum;
}

spVector 
spMouseJointGetAnchor(spConstraint* constraint)
{
    return mouseJoint->anchor;
}

spVector 
spMouseJointGetWorldAnchor(spConstraint* constraint)
{
    return spMult(mouseJoint->constraint.bodyA->xf, mouseJoint->anchor);
}

spVector 
spMouseJointGetTarget(spConstraint* constraint)
{
    return mouseJoint->target;
}

spVector 
spMouseJointGetLocalTarget(spConstraint* constraint)
{
    return spTMult(mouseJoint->constraint.bodyA->xf, mouseJoint->target);
}

spFloat 
spMouseJointGetFrequency(spConstraint* constraint)
{
    return mouseJoint->frequency;
}

spFloat 
spMouseJointGetDamping(spConstraint* constraint)
{
    return mouseJoint->damping;
}

void 
spMouseJointSetAnchor(spConstraint* constraint, spVector anchor)
{
    mouseJoint->anchor = anchor;
}

void 
spMouseJointSetWorldAnchor(spConstraint* constraint, spVector anchor)
{
    mouseJoint->anchor = spTMult(mouseJoint->constraint.bodyA->xf, anchor);
}

void 
spMouseJointSetTarget(spConstraint* constraint, spVector target)
{
    mouseJoint->target = target;
}

void 
spMouseJointSetLocalTarget(spConstraint* constraint, spVector target)
{
    mouseJoint->target = spMult(mouseJoint->constraint.bodyA->xf, target);
}

void 
spMouseJointSetFrequency(spConstraint* constraint, spFloat frequency)
{
    mouseJoint->frequency = frequency;
}

void 
spMouseJointSetDamping(spConstraint* constraint, spFloat damping)
{
    mouseJoint->damping = damping;
}