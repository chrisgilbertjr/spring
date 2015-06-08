
#include "spDistanceJoint.h"
#include "spDebugDraw.h"
#include "spBody.h"
#include "spSolver.h"

void 
spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchor_a, spVector anchor_b, spFloat distance)
{
    joint->base_class = spConstraintConstruct(a, b, SP_DISTANCE_CONSTRAINT);
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
    //spFree(joint);
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spDistanceJointPreStep(spDistanceJoint* joint, const spFloat h)
{
    /// TODO;
}

void 
spDistanceJointSolve(spDistanceJoint* joint)
{
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
    spDebugDrawLine(pa, pb, spRed(1.0f));
    spFloat length = spLength(spSub(pa, pb));
    spLog("%.7f\n", length);

    spVector n = spSub(pa, pb);
    spNormalize(&n);
    spVector rv = spRelativeVelocity(va, vb, wa, wb, ra, rb);
    spFloat nrv = spDot(n, rv);
    spFloat iem = spDistanceJointInvEffectiveMass(ima, imb, iia, iib, ra, rb, n);

    spFloat lambda = -nrv * iem;
    spVector P = spMult(n, lambda);
    ba->v  = spSub(ba->v, spMult(P, ima));
    bb->v  = spAdd(bb->v, spMult(P, imb));
    ba->w -= iia * spCross(ra, P);
    bb->w += iib * spCross(rb, P);
}

void 
spDistanceJointStabilize(spDistanceJoint* joint)
{
    /// TODO;
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
    spFloat length = spLength(n);
    spNormalize(&n);
    spFloat C = length - joint->distance;
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