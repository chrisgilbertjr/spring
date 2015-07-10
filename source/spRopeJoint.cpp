
#include "spRopeJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define ropeJoint spConstraintCastRopeJoint(constraint)


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

    /// compute the position constraint, and compute baumgarte velocity bias
    spFloat C = length - joint->maxDistance;
    joint->bias = C * (spBaumgarte / h);
}

static void 
WarmStart(spRopeJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute impulse
    spVector impulseB = spMult(joint->n, joint->lambdaAccum);
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
    /// get the bodies
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

    /// compute the impulses
    spVector impulseA = spMult(joint->n, joint->lambdaAccum - lambdaOld);
    spVector impulseB = spNegative(impulseA);

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
    return spMult(ropeJoint->constraint.bodyA->xf, ropeJoint->anchorA);
}

spVector 
spRopeJointGetWorldAnchorB(spConstraint* constraint)
{
    return spMult(ropeJoint->constraint.bodyB->xf, ropeJoint->anchorB);
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
    ropeJoint->anchorA = spTMult(ropeJoint->constraint.bodyA->xf, anchorA);
}

void 
spRopeJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    ropeJoint->anchorB = spTMult(ropeJoint->constraint.bodyB->xf, anchorB);
}

void 
spRopeJointSetMaxDistance(spConstraint* constraint, spFloat maxDistance)
{
    ropeJoint->maxDistance = maxDistance;
}
