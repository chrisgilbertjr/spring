
#include "spDistanceJoint.h"
#include "spDebugDraw.h"
#include "spBody.h"
#include "spSolver.h"

void 
spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance)
{
    joint->base_class = spConstraintConstruct(a, b, SP_DISTANCE_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->distance = distance;
}

spDistanceJoint* 
spDistanceConstraintAlloc()
{
    return (spDistanceJoint*) spMalloc(sizeof(spDistanceJoint));
}

spDistanceJoint* 
spDistanceConstraintNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance)
{
    spDistanceJoint* joint = spDistanceConstraintAlloc();
    spDistanceJointInit(joint, a, b, anchorA, anchorB, distance);
    return joint;
}

void 
spDistanceJointFree(spDistanceJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spDistanceJointPreStep(spDistanceJoint* joint, const spFloat h)
{
    spBody* bA = joint->base_class.body_a;
    spBody* bB = joint->base_class.body_b;
    spVector pA = spMult(bA->xf, joint->anchorA);
    spVector pB = spMult(bB->xf, joint->anchorB);

    joint->rA = spSub(pA, bA->p);
    joint->rB = spSub(pB, bB->p);
    joint->n = spMult(joint->n, 1.0f / (spLength(joint->n) + SP_FLT_EPSILON));

    spFloat nrA = spCross(joint->n, joint->rA);
    spFloat nrB = spCross(joint->n, joint->rB);
    joint->eMass =  1.0f / (bA->mInv + bB->mInv + bA->iInv * nrA * nrA + bB->iInv * nrB * nrB);
    joint->jAccum = 0.0f;
}

void 
spDistanceJointSolve(spDistanceJoint* joint)
{
    spBody* bA = joint->base_class.body_a;
    spBody* bB = joint->base_class.body_b;

    spVector rv = spRelativeVelocity(bA->v, bB->v, bA->w, bB->w, joint->rA, joint->rB);
    spFloat nrv = spDot(joint->n, rv);

    spFloat lambda = -nrv * joint->eMass;
    spFloat jPrev = joint->jAccum;
    joint->jAccum = jPrev + lambda;
    spFloat  j = joint->jAccum - jPrev;
    spVector P = spMult(joint->n, j);
    bA->v  = spSub(bA->v, spMult(P, bA->mInv));
    bB->v  = spAdd(bB->v, spMult(P, bB->mInv));
    bA->w -= bA->iInv * spCross(joint->rA, P);
    bB->w += bB->iInv * spCross(joint->rB, P);
}

spFloat 
spDistanceJointInvEffectiveMass(
    spFloat ima, 
    spFloat imb, 
    spFloat iia, 
    spFloat iib, 
    spVector ra, 
    spVector rb, 
    spVector n)
{
    spFloat nra = spCross(n, ra);
    spFloat nrb = spCross(n, rb);
    spFloat em = ima + imb + iia * nra * nra + iib * nrb * nrb;
    return 1.0f / em;
}