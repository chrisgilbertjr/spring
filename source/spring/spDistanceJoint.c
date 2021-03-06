
#include "spDistanceJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define distanceJoint spConstraintCastDistanceJoint(constraint)

static void 
Free(spDistanceJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spDistanceJoint* joint, const spFloat h)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute anchors in world space
    spVector anchorA = spxTransform(a->xf, joint->anchorA);
    spVector anchorB = spxTransform(b->xf, joint->anchorB);

    /// compute relative velocity
    joint->rA = spvSub(anchorA, a->p);
    joint->rB = spvSub(anchorB, b->p);

    /// compute the normal
    joint->n = spvSub(anchorA, anchorB);
    spFloat length = spvLength(joint->n);
    joint->n = spvfMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    /// compute the effective mass
    spFloat nrA = spvCross(joint->n, joint->rA);
    spFloat nrB = spvCross(joint->n, joint->rB);
    joint->eMass = a->mInv + b->mInv + a->iInv * nrA * nrA + b->iInv * nrB * nrB;
    joint->eMass = joint->eMass ? 1.0f / joint->eMass : 0.0f;

    /// compute the position constraint, and compute baumgarte velocity bias
    spFloat C = length - joint->distance;
    joint->bias = C * (spBaumgarte / h);
}

static void 
WarmStart(spDistanceJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute impulse
    spVector impulseB = spvfMult(joint->n, joint->lambdaAccum);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);

    /// reset the lagrange multiplier
    joint->lambdaAccum = 0.0f;
}

static void 
Solve(spDistanceJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the velocity constraint
    spVector rvA = spvAdd(a->v, spfvCross(a->w, joint->rA));
    spVector rvB = spvAdd(b->v, spfvCross(b->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spvSub(rvB, rvA));

    /// accumulate the impulse
    spFloat lambda = (joint->bias - Cdot) * joint->eMass;
    joint->lambdaAccum += lambda;

    /// compute the impulses
    spVector impulseB = spvfMult(joint->n, lambda);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);
}

void 
spDistanceJointInit(spDistanceJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance)
{
    joint->constraint = spConstraintConstruct(a, b, SP_DISTANCE_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->rA = spVectorZero();
    joint->rB = spVectorZero();
    joint->n  = spVectorZero();
    joint->lambdaAccum = 0.0f;
    joint->distance = distance;
    joint->eMass = 0.0f;
    joint->bias = 0.0f;
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
}

spDistanceJoint* 
spDistanceJointAlloc()
{
    return (spDistanceJoint*) spMalloc(sizeof(spDistanceJoint));
}

spConstraint* 
spDistanceJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance)
{
    spDistanceJoint* joint = spDistanceJointAlloc();
    NULLCHECK(joint);
    spDistanceJointInit(joint, a, b, anchorA, anchorB, distance);
    return (spConstraint*) joint;
}

spBool 
spConstraintIsDistanceJoint(spConstraint* constraint)
{
    return constraint->type == SP_DISTANCE_JOINT;
}

spDistanceJoint* 
spConstraintCastDistanceJoint(spConstraint* constraint)
{
    if (spConstraintIsDistanceJoint(constraint))
    {
        return (spDistanceJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a distance joint\n");
        return NULL;
    }
}

spFloat 
spDistanceJointGetImpulse(spConstraint* constraint)
{
    return distanceJoint->lambdaAccum;
}

spVector 
spDistanceJointGetAnchorA(spConstraint* constraint)
{
    return distanceJoint->anchorA;
}

spVector 
spDistanceJointGetAnchorB(spConstraint* constraint)
{
    return distanceJoint->anchorB;
}

spVector 
spDistanceJointGetWorldAnchorA(spConstraint* constraint)
{
    return spxTransform(constraint->bodyA->xf, distanceJoint->anchorA);
}

spVector 
spDistanceJointGetWorldAnchorB(spConstraint* constraint)
{
    return spxTransform(constraint->bodyB->xf, distanceJoint->anchorB);
}

spFloat 
spDistanceJointGetDistance(spConstraint* constraint)
{
    return distanceJoint->distance;
}

void 
spDistanceJointSetAnchorA(spConstraint* constraint, spVector anchorA)
{
    distanceJoint->anchorA = anchorA;
}

void 
spDistanceJointSetAnchorB(spConstraint* constraint, spVector anchorB)
{
    distanceJoint->anchorB = anchorB;
}

void 
spDistanceJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA)
{
    distanceJoint->anchorA = spxTTransform(constraint->bodyA->xf, anchorA);
}

void 
spDistanceJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    distanceJoint->anchorB = spxTTransform(constraint->bodyB->xf, anchorB);
}

void 
spDistanceJointSetDistance(spConstraint* constraint, spFloat distance)
{
    distanceJoint->distance = distance;
}