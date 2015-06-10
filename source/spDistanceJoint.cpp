
#include "spDistanceJoint.h"
#include "spDebugDraw.h"
#include "spBody.h"
#include "spSolver.h"

void 
spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance)
{
    joint->base_class = spConstraintConstruct(a, b, SP_DISTANCE_JOINT);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    joint->distance = distance;
}

spDistanceJoint* 
spDistanceConstraintAlloc()
{
    return (spDistanceJoint*) spMalloc(sizeof(spDistanceJoint));
}

spDistanceJoint* 
spDistanceConstraintNew(spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance)
{
    spDistanceJoint* joint = spDistanceConstraintAlloc();
    spDistanceJointInit(joint, a, b, anchor_a, anchor_b, distance);
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
    spVector pA = spMult(bA->xf, joint->anchor_a);
    spVector pB = spMult(bB->xf, joint->anchor_b);

    joint->rA = spSub(pA, bA->p);
    joint->rB = spSub(pB, bB->p);
    joint->n = spNormal(spSub(pA, pB));

    spFloat nrA = spCross(joint->n, joint->rA);
    spFloat nrB = spCross(joint->n, joint->rB);
    joint->eMass =  1.0f / (bA->m_inv + bB->m_inv + bA->i_inv * nrA * nrA + bB->i_inv * nrB * nrB);
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
    bA->v  = spSub(bA->v, spMult(P, bA->m_inv));
    bB->v  = spAdd(bB->v, spMult(P, bB->m_inv));
    bA->w -= bA->i_inv * spCross(joint->rA, P);
    bB->w += bB->i_inv * spCross(joint->rB, P);
}

void 
spDistanceJointStabilize(spDistanceJoint* joint)
{
    /// TODO - clean this up
    spConstraint constraint = joint->base_class;
    spBody* ba = constraint.body_a;
    spBody* bb = constraint.body_b;
    spTransform* xfa = &ba->xf;
    spTransform* xfb = &bb->xf;
    spVector pa = spMult(*xfa, joint->anchor_a);
    spVector pb = spMult(*xfb, joint->anchor_b);
    spVector ra = spSub(pa, spMult(*xfa, ba->p));
    spVector rb = spSub(pb, spMult(*xfb, bb->p));
    spVector va = ba->v;
    spVector vb = bb->v;
    spFloat wa = ba->w;
    spFloat wb = bb->w;
    spFloat ima = ba->m_inv;
    spFloat imb = bb->m_inv;
    spFloat iia = ba->i_inv;
    spFloat iib = bb->i_inv;

    spVector n = spSub(pa, pb);
    spDebugDrawLine(pa, pb, spGreen(1.0f));
    spFloat length = spLength(n);
    spLog("%.7f\n", length);
    spFloat C = length - joint->distance;
    n = spMult(n, 1.0f / (length + SP_FLT_EPSILON));
    spVector P = spMult(n, C);
    ba->p = spSub(ba->p, spMult(P, ima));
    bb->p = spAdd(bb->p, spMult(P, imb));
    ba->a -= iia * spCross(ra, P);
    bb->a += iib * spCross(rb, P);
    __spBodyUpdateTransform(ba);
    __spBodyUpdateTransform(bb);
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