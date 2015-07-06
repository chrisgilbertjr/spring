
#include "spRopeJoint.h"
#include "spBody.h"

spVector 
spRopeJointGetLocalAnchorA(spRopeJoint* joint)
{
    return joint->anchorA;
}

spVector 
spRopeJointGetLocalAnchorB(spRopeJoint* joint)
{
    return joint->anchorB;
}

spVector 
spRopeJointGetWorldAnchorA(spRopeJoint* joint)
{
    return spMult(joint->constraint.bodyA->xf, joint->anchorA);
}

spVector 
spRopeJointGetWorldAnchorB(spRopeJoint* joint)
{
    return spMult(joint->constraint.bodyB->xf, joint->anchorB);
}

spVector 
spRopeJointGetImpulseDirection(spRopeJoint* joint)
{
    return joint->n;
}

spVector 
spRopeJointGetImpulse(spRopeJoint* joint)
{
    return spMult(joint->n, joint->lambdaAccum);
}

spFloat 
spRopeJointGetMaxDistance(spRopeJoint* joint)
{
    return joint->maxDistance;
}

spFloat 
spRopeJointGetImpulseLength(spRopeJoint* joint)
{
    return joint->lambdaAccum;
}

void 
spRopeJointSetLocalAnchorA(spRopeJoint* joint, spVector anchorA)
{
    joint->anchorA = anchorA;
}

void 
spRopeJointSetLocalAnchorB(spRopeJoint* joint, spVector anchorB)
{
    joint->anchorB = anchorB;
}

void 
spRopeJointSetWorldAnchorA(spRopeJoint* joint, spVector anchorA)
{
    joint->anchorA = spTMult(joint->constraint.bodyA->xf, anchorA);
}

void 
spRopeJointSetWorldAnchorB(spRopeJoint* joint, spVector anchorB)
{
    joint->anchorB = spTMult(joint->constraint.bodyB->xf, anchorB);
}

void 
spRopeJointSetMaxDistance(spRopeJoint* joint, spFloat maxDistance)
{
    joint->maxDistance = maxDistance;
}

void 
spRopeJointInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    joint->constraint = spConstraintConstruct(a, b, SP_ROPE_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->rA = spVectorZero();
    joint->rB = spVectorZero();
    joint->n = spVectorZero();
    joint->maxDistance = maxDistance;
    joint->lambdaAccum = 0.0f;
    joint->eMass = 0.0f;
    joint->bias = 0.0f;
}

void 
spRopeJointWorldInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    spRopeJointInit(joint, a, b, spTMult(a->xf, anchorA), spTMult(b->xf, anchorB), maxDistance);
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

spRopeJoint* 
spRopeJointWorldNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    spRopeJoint* joint = spRopeJointAlloc();
    spRopeJointWorldInit(joint, a, b, anchorA, anchorB, maxDistance);
    return joint;
}

void 
spRopeJointFree(spRopeJoint** joint)
{
    free(*joint);
    *joint = NULL;
}

void 
spRopeJointApplyCachedImpulse(spRopeJoint* joint, const spFloat h)
{
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute impulse
    spVector impulse = spMult(joint->n, joint->lambdaAccum);

    /// apply the impulse
    a->v = spSub(a->v, spMult(impulse, a->mInv));
    b->v = spAdd(b->v, spMult(impulse, b->mInv));
    a->w -= a->iInv * spCross(joint->rA, impulse);
    b->w += b->iInv * spCross(joint->rB, impulse);
}

void 
spRopeJointPreSolve(spRopeJoint* joint, const spFloat h)
{
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute anchors in world space
    spVector anchorA = spMult(a->xf, joint->anchorA);
    spVector anchorB = spMult(b->xf, joint->anchorB);

    /// compute relative velocity and normal direction
    joint->rA = spSub(anchorA, a->p);
    joint->rB = spSub(anchorB, b->p);
    joint->n = spSub(anchorA, anchorB);
    spFloat length = spLength(joint->n);
    joint->n = spMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    /// compute the effective mass
    spFloat nrA = spCross(joint->n, joint->rA);
    spFloat nrB = spCross(joint->n, joint->rB);
    joint->eMass = a->mInv + b->mInv + a->iInv * nrA * nrA + b->iInv * nrB * nrB;
    joint->eMass = joint->eMass ? 1.0f / joint->eMass : 0.0f;

    /// compute the position constraint, and compute baumgarte stabilization bias
    spFloat C = length - joint->maxDistance;
    spFloat beta = 0.2f;
    joint->bias = C * (beta / h);
    joint->lambdaAccum = 0.0f;
}

void 
spRopeJointSolve(spRopeJoint* joint)
{
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the velocity constraint
    spVector rvA = spAdd(a->v, spCross(a->w, joint->rA));
    spVector rvB = spAdd(b->v, spCross(b->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spSub(rvB, rvA));

    /// accumulate the impulse
    spFloat lambda = (joint->bias - Cdot) * joint->eMass;
    spFloat lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum += lambda;
    spVector impulse = spMult(joint->n, joint->lambdaAccum - lambdaOld);

    /// apply the impulse
    a->v = spSub(a->v, spMult(impulse, a->mInv));
    b->v = spAdd(b->v, spMult(impulse, b->mInv));
    a->w -= a->iInv * spCross(joint->rA, impulse);
    b->w += b->iInv * spCross(joint->rB, impulse);
}