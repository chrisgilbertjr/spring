
#include "spDebugDraw.h"
#include "spRopeJoint.h"
#include "spSolver.h"
#include "spBody.h"

void 
spRopeJointInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    joint->constraint = spConstraintConstruct(a, b, SP_ROPE_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->maxDistance = maxDistance;
    joint->shorterThanMax = false;
    joint->bias = 0.0f;
}

spRopeJoint* 
spRopeJointAlloc()
{
    return (spRopeJoint*) spMalloc(sizeof(spRopeJoint));
}

spRopeJoint* 
spRopeJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    spRopeJoint* joint = spRopeJointAlloc();
    spRopeJointInit(joint, a, b, anchorA, anchorB, maxDistance);
    return joint;
}

void 
spRopeJointFree(spRopeJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spRopeJointPreStep(spRopeJoint* joint, const spFloat h)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;
    spVector pA = spMult(bA->xf, joint->anchorA);
    spVector pB = spMult(bB->xf, joint->anchorB);

    joint->rA = spSub(pA, bA->p);
    joint->rB = spSub(pB, bB->p);
    joint->n = spSub(pA, pB);
    spFloat length = spLength(joint->n);
    joint->n = spMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    spFloat nrA = spCross(joint->n, joint->rA);
    spFloat nrB = spCross(joint->n, joint->rB);
    joint->eMass = 1.0f / (bA->m_inv + bB->m_inv + bA->i_inv * nrA * nrA + bB->i_inv * nrB * nrB);
    joint->jAccum = 0.0f;
    spFloat bias = powf(1.0f - .1f, 60.0f);
    bias = 1.0f - powf(bias, h);
    joint->bias = .2f * (length - joint->maxDistance) / h;
}

void 
spRopeJointSolve(spRopeJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    spVector rv = spRelativeVelocity(bA->v, bB->v, bA->w, bB->w, joint->rA, joint->rB);
    spFloat nrv = spDot(joint->n, rv);

    spFloat lambda = joint->bias - nrv * joint->eMass;
    spFloat jPrev = joint->jAccum;
    joint->jAccum = jPrev + lambda;
    spFloat  j = joint->jAccum - jPrev;
    spVector P = spMult(joint->n, j);
    bA->v = spSub(bA->v, spMult(P, bA->m_inv));
    bB->v = spAdd(bB->v, spMult(P, bB->m_inv));
    bA->w -= bA->i_inv * spCross(joint->rA, P);
    bB->w += bB->i_inv * spCross(joint->rB, P);
}

void 
spRopeJointStabilize(spRopeJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;
    spVector pA = spMult(bA->xf, joint->anchorA);
    spVector pB = spMult(bB->xf, joint->anchorB);
    spDebugDrawLine(pA, pB, spGreen(1.0f));
    //spVector n = spSub(pA, pB);
    //spFloat length = spLength(n);
    //spFloat C = length - joint->maxDistance;
    //spVector P = spMult(C, joint->n);

    //bA->p = spSub(bA->p, spMult(P, bA->m_inv));
    //bB->p = spAdd(bB->p, spMult(P, bB->m_inv));
    //bA->a -= bA->i_inv * spCross(joint->rA, P);
    //bB->a += bB->i_inv * spCross(joint->rB, P);
    //__spBodyUpdateTransform(bA);
    //__spBodyUpdateTransform(bB);
}