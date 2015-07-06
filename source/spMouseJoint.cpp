
#include "spDebugDraw.h"
#include "spMouseJoint.h"
#include "spBody.h"

void 
spMouseJointInit(spMouseJoint* joint, spBody* a, spFloat frequency, spFloat damping, spVector anchor, spVector target)
{
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
    spMouseJointInit(joint, a, frequency, damping, anchor, target);
    return joint;
}

void 
spMouseJointFree(spMouseJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spMouseJointPreSolve(spMouseJoint* joint, const spFloat h)
{
    spBody* a = joint->constraint.bodyA;
    
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * a->m * joint->damping * omega;
    spFloat k = a->m * omega * omega;

    spFloat hk = h * k;
    joint->gamma = h * (c + hk);
    joint->gamma = joint->gamma ? 1.0f / joint->gamma : 0.0f;
    spFloat beta = hk * joint->gamma;

    spVector local = spVectorZero();
    joint->rA = spMult(a->xf.q, spSub(joint->anchor, a->com));

    spMatrix K = spMatrixZero();

    K.b = K.c = -a->iInv * joint->rA.x * joint->rA.y;
    K.a = a->mInv + a->iInv * joint->rA.y * joint->rA.y + joint->gamma;
    K.d = a->mInv + a->iInv * joint->rA.x * joint->rA.x + joint->gamma;

    joint->eMass = spInverse(K);

    a->w *= .98f;

    spVector C = spAdd(a->p, spSub(joint->rA, joint->target));
    joint->bias = spMult(C, beta);
    joint->lambdaAccum = spVectorZero();

    spDebugDrawPoint(spMult(a->xf, joint->anchor), spGreen(1.0f));
    spDebugDrawPoint(joint->target, spRed(1.0f));
    spDebugDrawLine(spMult(a->xf, joint->anchor), joint->target, spGreen(1.0f));
}

void 
spMouseJointSolve(spMouseJoint* joint)
{
    spBody* a = joint->constraint.bodyA;

    spVector Cdot = spAdd(a->v, spCross(a->w, joint->rA));
    spVector A = spAdd(Cdot, joint->bias);
    spVector B = spMult(joint->gamma, joint->lambdaAccum);
    spVector D = spNegate(spAdd(A, B));
    spVector lambda = spMult(joint->eMass, D);
    spVector lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum = spAdd(joint->lambdaAccum, lambda);
    spVector impulse = spSub(joint->lambdaAccum, lambdaOld);

    a->v  = spAdd(a->v, spMult(a->mInv, impulse));
    a->w += a->iInv * spCross(joint->rA, impulse);
}