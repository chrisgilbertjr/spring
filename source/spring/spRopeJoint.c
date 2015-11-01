
#include "spRopeJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define ropeJoint spConstraintCastRopeJoint(constraint)

spFloat spRopeJointDistBias = 0.0f;

static void 
Free(spRopeJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spRopeJoint* joint, const spFloat h)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute anchors in world space
    spVector anchorA = spMultXformVec(a->xf, joint->anchorA);
    spVector anchorB = spMultXformVec(b->xf, joint->anchorB);

    /// compute relative velocity
    joint->rA = spSubVecs(anchorA, a->p);
    joint->rB = spSubVecs(anchorB, b->p);

    /// compute the normal
    joint->n = spSubVecs(anchorA, anchorB);
    spFloat length = spLength(joint->n);
    if (length <= (joint->maxDistance + spRopeJointDistBias))
    {
        joint->n = spVectorZero();
        return;
    }
    joint->n = spMultVecFlt(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    /// compute the effective mass
    spFloat nrA = spCrossVecs(joint->n, joint->rA);
    spFloat nrB = spCrossVecs(joint->n, joint->rB);
    joint->eMass = a->mInv + b->mInv + a->iInv * nrA * nrA + b->iInv * nrB * nrB;
    joint->eMass = joint->eMass ? 1.0f / joint->eMass : 0.0f;

    /// compute the position constraint, and compute baumgarte velocity bias
    spFloat C = length - joint->maxDistance;
    joint->bias = C * (spBaumgarte / h);
    joint->lambdaAccum = 0.0f;
}

static void 
WarmStart(spRopeJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute impulse
    spVector impulseB = spMultVecFlt(joint->n, joint->lambdaAccum);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);

    /// reset the lagrange multiplier
    joint->lambdaAccum = 0.0f;
}

static void 
Solve(spRopeJoint* joint)
{
    /// exit if the distance was closer than the max distance
    if (spEqual(joint->n, spVectorZero())) { return; }

    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the velocity constraint
    spVector rvA = spAddVecs(a->v, spCross(a->w, joint->rA));
    spVector rvB = spAddVecs(b->v, spCross(b->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spSubVecs(rvB, rvA));

    /// accumulate the impulse
    spFloat lambda = (joint->bias - Cdot) * joint->eMass;
    joint->lambdaAccum += lambda;

    /// compute the impulses
    spVector impulseB = spMultVecFlt(joint->n, lambda);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);
}

void 
spRopeJointInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    NULLCHECK(joint); NULLCHECK(a); NULLCHECK(b);
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
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
}

void 
spRopeJointWorldInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    spRopeJointInit(joint, a, b, spTMultXformVec(a->xf, anchorA), spTMultXformVec(b->xf, anchorB), maxDistance);
}

spRopeJoint* 
spRopeJointAlloc()
{
    return (spRopeJoint*) spMalloc(sizeof(spRopeJoint));
}

spConstraint* 
spRopeJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    spRopeJoint* joint = spRopeJointAlloc();
    spRopeJointInit(joint, a, b, anchorA, anchorB, maxDistance);
    return (spConstraint*)joint;
}

spRopeJoint* 
spRopeJointWorldNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance)
{
    spRopeJoint* joint = spRopeJointAlloc();
    NULLCHECK(joint);
    spRopeJointWorldInit(joint, a, b, anchorA, anchorB, maxDistance);
    return joint;
}


spBool 
spConstraintIsRopeJoint(spConstraint* constraint)
{
    return constraint->type == SP_ROPE_JOINT;
}

spRopeJoint* 
spConstraintCastRopeJoint(spConstraint* constraint)
{
    if (spConstraintIsRopeJoint(constraint))
    {
        return (spRopeJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a rope joint\n");
        return NULL;
    }
}

spFloat 
spRopeJointGetImpulse(spConstraint* constraint)
{
    return ropeJoint->lambdaAccum;
}

spVector 
spRopeJointGetAnchorA(spConstraint* constraint)
{
    return ropeJoint->anchorA;
}

spVector 
spRopeJointGetAnchorB(spConstraint* constraint)
{
    return ropeJoint->anchorB;
}

spVector 
spRopeJointGetWorldAnchorA(spConstraint* constraint)
{
    return spMultXformVec(ropeJoint->constraint.bodyA->xf, ropeJoint->anchorA);
}

spVector 
spRopeJointGetWorldAnchorB(spConstraint* constraint)
{
    return spMultXformVec(ropeJoint->constraint.bodyB->xf, ropeJoint->anchorB);
}

spFloat 
spRopeJointGetMaxDistance(spConstraint* constraint)
{
    return ropeJoint->maxDistance;
}

void 
spRopeJointSetAnchorA(spConstraint* constraint, spVector anchorA)
{
    ropeJoint->anchorA = anchorA;
}

void 
spRopeJointSetAnchorB(spConstraint* constraint, spVector anchorB)
{
    ropeJoint->anchorB = anchorB;
}

void 
spRopeJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA)
{
    ropeJoint->anchorA = spTMultXformVec(ropeJoint->constraint.bodyA->xf, anchorA);
}

void 
spRopeJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    ropeJoint->anchorB = spTMultXformVec(ropeJoint->constraint.bodyB->xf, anchorB);
}

void 
spRopeJointSetMaxDistance(spConstraint* constraint, spFloat maxDistance)
{
    ropeJoint->maxDistance = maxDistance;
}
