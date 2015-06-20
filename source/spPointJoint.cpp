
#include "spPointJoint.h"
#include "spBody.h"
#include "spDebugDraw.h"

void 
spPointJointInit(spPointJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB)
{
    joint->constraint = spConstraintConstruct(a, b, SP_POINT_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->lambdaAccum = spVectorZero();
    joint->bias = spVectorZero();
    joint->rA = spVectorZero();
    joint->rB = spVectorZero();
    joint->eMass = spMatrixIdentity();
}

spPointJoint* 
spPointJointAlloc()
{
    return (spPointJoint*) spMalloc(sizeof(spPointJoint));
}

spPointJoint* 
spPointJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB)
{
    spPointJoint* joint = spPointJointAlloc();
    spPointJointInit(joint, a, b, anchorA, anchorB);
    return joint;
}

void 
spPointJointFree(spPointJoint** joint)
{
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spPointJointPreSolve(spPointJoint* joint, const spFloat h)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;
    spVector pA = spMult(bA->xf, joint->anchorA);
    spVector pB = spMult(bB->xf, joint->anchorB);

    spVector rA = joint->rA = spSub(pA, bA->p);
    spVector rB = joint->rB = spSub(pB, bB->p);

    spDebugDrawLine(pA, bA->p, spGreen(1.0f));
    spDebugDrawLine(pB, bB->p, spGreen(1.0f));
    spDebugDrawPoint(pB, spYellow(1.0f));

    /// compute the effective mass matrix
    spFloat iiA = bA->i_inv;
    spFloat iiB = bB->i_inv;
    spFloat iMass = bA->m_inv + bB->m_inv;

    spMatrix K = spMatrix(iMass, 0.0f, 0.0f, iMass);

    spFloat yiA  =  rA.y * iiA;
    spFloat yiB  =  rB.y * iiB;
    spFloat rABn = -rA.x * yiA + -rB.x * yiB;

    K.a += rA.y * yiA + rB.y * yiB;
    K.b += rABn;
    K.c += rABn;
    K.d += rA.x * rA.x * iiA + rB.x * rB.x * iiB;

    joint->eMass = spInverse(K);

    /// compute bias velocity given the position constraint
    spVector C = spSub(pB, pA);
    spFloat beta = 0.3f;
    joint->bias = spMult(C, -beta / h); 
}

void 
spPointJointSolve(spPointJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    spVector Cdot = spSub(spAdd(bB->v, spCross(bB->w, joint->rB)), spAdd(bA->v, spCross(bA->w, joint->rA)));

    spVector lambda = spMult(joint->eMass, spSub(joint->bias, Cdot));
    spVector lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum = spAdd(joint->lambdaAccum, lambda);
    spVector impulse = spSub(joint->lambdaAccum, lambdaOld);

    bA->v  = spSub(bA->v, spMult(impulse, bA->m_inv));
    bB->v  = spAdd(bB->v, spMult(impulse, bB->m_inv));
    bA->w -= bA->i_inv * spCross(joint->rA, impulse);
    bB->w += bB->i_inv * spCross(joint->rB, impulse);
}