
#include "spPointJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define pointJoint spConstraintCastPointJoint(constraint)

static void 
Free(spPointJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spPointJoint* joint, const spFloat h)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute world anchors
    spVector pA = spxTransform(a->xf, joint->anchorA);
    spVector pB = spxTransform(b->xf, joint->anchorB);

    /// compute rel velocity
    spVector rA = joint->rA = spvSub(pA, a->p);
    spVector rB = joint->rB = spvSub(pB, b->p);

    /// compute the effective mass matrix
    spFloat iiA = a->iInv;
    spFloat iiB = b->iInv;
    spFloat iMass = a->mInv + b->mInv;

    spMatrix K = spMatrixConstruct(iMass, 0.0f, 0.0f, iMass);

    spFloat yiA  =  rA.y * iiA;
    spFloat yiB  =  rB.y * iiB;
    spFloat rABn = -rA.x * yiA + -rB.x * yiB;

    K.a += rA.y * yiA + rB.y * yiB;
    K.b += rABn;
    K.c += rABn;
    K.d += rA.x * rA.x * iiA + rB.x * rB.x * iiB;

    /// invert the matrix to get the effective mass
    joint->eMass = spmInverse(K);

    /// compute the position constraint and the baumgarte velocity bias
    spVector C = spvSub(pB, pA);
    joint->bias = spvfMult(C, -spBaumgarte / h); 
}

static void 
WarmStart(spPointJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// get the impulses
    spVector impulseB = joint->lambdaAccum;
    spVector impulseA = spNegative(impulseB);

    /// apply the impulses
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);

    /// reset the lagrange multiplier
    joint->lambdaAccum = spVectorZero();
}

static void 
Solve(spPointJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the velocity constraint
    spVector Cdot = spvSub(spvAdd(b->v, spfvCross(b->w, joint->rB)), spvAdd(a->v, spfvCross(a->w, joint->rA)));

    /// compute the lagrange multiplier
    spVector lambda = spmTransform(joint->eMass, spvSub(joint->bias, Cdot));
    spVector lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum = spvAdd(joint->lambdaAccum, lambda);

    /// compute the impulses
    spVector impulseB = spvSub(joint->lambdaAccum, lambdaOld);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulses
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);
}

void 
spPointJointInit(spPointJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB)
{
    NULLCHECK(joint);
    joint->constraint = spConstraintConstruct(a, b, SP_POINT_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->lambdaAccum = spVectorZero();
    joint->bias = spVectorZero();
    joint->rA = spVectorZero();
    joint->rB = spVectorZero();
    joint->eMass = spMatrixZero();
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
    int x = 0;
}

spPointJoint* 
spPointJointAlloc()
{
    return (spPointJoint*) spMalloc(sizeof(spPointJoint));
}

spConstraint* 
spPointJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB)
{
    spPointJoint* joint = spPointJointAlloc();
    NULLCHECK(joint);
    spPointJointInit(joint, a, b, anchorA, anchorB);
    return (spConstraint*) joint;
}

spBool 
spConstraintIsPointJoint(spConstraint* constraint)
{
    return constraint->type == SP_POINT_JOINT;
}

spPointJoint* 
spConstraintCastPointJoint(spConstraint* constraint)
{
    if (spConstraintIsPointJoint(constraint))
    {
        return (spPointJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a point joint\n");
        return NULL;
    }
}

spVector 
spPointJointGetImpulse(spConstraint* constraint)
{
    return pointJoint->lambdaAccum;
}

spVector 
spPointJointGetAnchorA(spConstraint* constraint)
{
    return pointJoint->anchorA;
}

spVector 
spPointJointGetAnchorB(spConstraint* constraint)
{
    return pointJoint->anchorB;
}

spVector 
spPointJointGetWorldAnchorA(spConstraint* constraint)
{
    return spxTransform(constraint->bodyA->xf, pointJoint->anchorA);
}

spVector 
spPointJointGetWorldAnchorB(spConstraint* constraint)
{
    return spxTransform(constraint->bodyB->xf, pointJoint->anchorB);
}

spVector 
spPointJointGetImpulseA(spConstraint* constraint)
{
    return pointJoint->lambdaAccum;
}

spVector 
spPointJointGetImpulseB(spConstraint* constraint)
{
    return spNegative(pointJoint->lambdaAccum);
}

spVector 
spPointJointGetRelVelocityA(spConstraint* constraint)
{
    return pointJoint->rA;
}

spVector 
spPointJointGetRelVelocityB(spConstraint* constraint)
{
    return pointJoint->rB;
}

void 
spPointJointSetAnchorA(spConstraint* constraint, spVector anchorA)
{
    pointJoint->anchorA = anchorA;
}

void 
spPointJointSetAnchorB(spConstraint* constraint, spVector anchorB)
{
    pointJoint->anchorB = anchorB;
}

void 
spPointJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA)
{
    pointJoint->anchorA = spxTTransform(constraint->bodyA->xf, anchorA);
}

void 
spPointJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    pointJoint->anchorB = spxTTransform(constraint->bodyB->xf, anchorB);
}