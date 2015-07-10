
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

    /// compute the world space anchors
    spVector pA = spMult(a->xf, joint->anchorA);
    spVector pB = spMult(b->xf, joint->anchorB);

    /// compute rel velocity
    joint->rA = spSub(pA, a->p);
    joint->rB = spSub(pB, b->p);

    /// compute normal
    spVector dir = spSub(pB, pA);
    spFloat length = spLength(dir);
    joint->n = spMult(dir, 1.0f / (length + SP_FLT_EPSILON));

    /// compute effective mass
    spFloat nrA = spCross(joint->n, joint->rA);
    spFloat nrB = spCross(joint->n, joint->rB);
    joint->eMass =  1.0f / (a->mInv + b->mInv + a->iInv * nrA * nrA + b->iInv * nrB * nrB);

    /// compute position constraint and baumgarte velocity bias
    spFloat C = length - joint->distance;
    joint->bias = -spBaumgarte * C / h;
}

static void 
WarmStart(spDistanceJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the impulses
    spVector impulseB = spMult(joint->n, joint->lambdaAccum);
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
    spVector rvA = spAdd(a->v, spCross(a->w, joint->rA));
    spVector rvB = spAdd(b->v, spCross(b->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spSub(rvB, rvA));

    /// compute the multiplier
    spFloat lambda = -(Cdot + joint->bias) * joint->eMass;
    spFloat lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum = lambdaOld + lambda;

    /// compute the impulse
    spVector impulseB = spMult(joint->n, joint->lambdaAccum - lambdaOld);
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
spDistanceConstraintAlloc()
{
    return (spDistanceJoint*) spMalloc(sizeof(spDistanceJoint));
}

spDistanceJoint* 
spDistanceConstraintNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat distance)
{
    spDistanceJoint* joint = spDistanceConstraintAlloc();
    NULLCHECK(joint);
    spDistanceJointInit(joint, a, b, anchorA, anchorB, distance);
    return joint;
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
spDistanceJointWorldGetAnchorA(spConstraint* constraint)
{
    return spMult(constraint->bodyA->xf, distanceJoint->anchorA);
}

spVector 
spDistanceJointWorldGetAnchorB(spConstraint* constraint)
{
    return spMult(constraint->bodyB->xf, distanceJoint->anchorB);
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
spDistanceJointWorldSetAnchorA(spConstraint* constraint, spVector anchorA)
{
    distanceJoint->anchorA = spMult(constraint->bodyA->xf, anchorA);
}

void 
spDistanceJointWorldSetAnchorB(spConstraint* constraint, spVector anchorB)
{
    distanceJoint->anchorB = spMult(constraint->bodyB->xf, anchorB);
}

void 
spDistanceJointSetDistance(spConstraint* constraint, spFloat distance)
{
    distanceJoint->distance = distance;
}