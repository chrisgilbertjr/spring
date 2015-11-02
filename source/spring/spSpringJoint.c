
#include "spSpringJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define springJoint spConstraintCastSpringJoint(constraint)

static void 
Free(spSpringJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spSpringJoint* joint, const spFloat h)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// get the anchors in world space
    spVector pA = spxTransform(a->xf, joint->anchorA);
    spVector pB = spxTransform(b->xf, joint->anchorB);

    /// calculate relative velocity and normal
    joint->rA = spvSub(pA, a->p);
    joint->rB = spvSub(pB, b->p);
    joint->n = spvSub(pB, pA);
    spFloat length = spvLength(joint->n);
    joint->n = spvfMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    /// compute inverse mass
    spFloat rcnA = spvCross(joint->rA, joint->n);
    spFloat rcnB = spvCross(joint->rB, joint->n);
    spFloat iMass = a->mInv + a->iInv * rcnA * rcnA + b->mInv + b->iInv * rcnB * rcnB;
    spFloat mass = 1.0f / (iMass + SP_FLT_EPSILON);

    /// compute position constraint
    spFloat C = length - joint->restLength;

    /// compute spring params
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * mass * joint->damping * omega;
    spFloat k = mass * omega * omega;
    joint->gamma = 1.0f / (h * (c + h * k) + SP_FLT_EPSILON);
    joint->bias = C * h * k * joint->gamma;

    /// compute effective mass
    iMass += joint->gamma;
    joint->eMass = 1.0f / (iMass + SP_FLT_EPSILON);
}

static void 
WarmStart(spSpringJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the impulses
    spVector impulseB = spvfMult(joint->n, joint->lambdaAccum);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);

    /// reset lagrange multiplier
    joint->lambdaAccum = 0.0f;
}

static void 
Solve(spSpringJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute velocity constraint
    spVector rvA = spvAdd(a->v, spfvCross(a->w, joint->rA));
    spVector rvB = spvAdd(b->v, spfvCross(b->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spvSub(rvB, rvA));

    /// compute lagrange multiplier
    spFloat lambda = -joint->eMass * (Cdot + joint->bias + joint->gamma * joint->lambdaAccum);
    joint->lambdaAccum += lambda;

    /// compute the impulse
    spVector impulseB = spvfMult(joint->n, lambda);
    spVector impulseA = spNegative(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);

}

void 
spSpringJointInit(
    spSpringJoint* joint, 
	spBody*        a, 
	spBody*        b, 
	spVector       anchorA, 
	spVector       anchorB, 
	spFloat        frequency, 
	spFloat        damping, 
	spFloat        restLength)
{
    joint->constraint = spConstraintConstruct(a, b, SP_SPRING_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->rA = spVectorZero();
    joint->rB = spVectorZero();
    joint->n = spVectorZero();
    joint->lambdaAccum = 0.0f;
    joint->restLength = restLength;
    joint->frequency = frequency;
    joint->damping = damping;
    joint->gamma = 0.0f;
    joint->bias = 0.0f;
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
}

spSpringJoint* 
spSpringJointAlloc()
{
    return (spSpringJoint*) spMalloc(sizeof(spSpringJoint));
}

spConstraint* 
spSpringJointNew(
	spBody*        a, 
	spBody*        b, 
	spVector       anchorA, 
	spVector       anchorB, 
	spFloat        frequency, 
	spFloat        damping, 
	spFloat        restLength)
{
    spSpringJoint* spring = spSpringJointAlloc();
    NULLCHECK(spring);
    spSpringJointInit(spring, a, b, anchorA, anchorB, frequency, damping, restLength);
    return (spConstraint*) spring;
}

spBool 
spConstraintIsSpringJoint(spConstraint* constraint)
{
    return constraint->type == SP_SPRING_JOINT;
}

spSpringJoint* 
spConstraintCastSpringJoint(spConstraint* constraint)
{
    if (spConstraintIsSpringJoint(constraint))
    {
        return (spSpringJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a spring joint\n");
        return NULL;
    }
}

spFloat 
spSpringJointGetImpulse(spConstraint* constraint)
{
    return springJoint->lambdaAccum;
}

spVector 
spSpringJointGetAnchorA(spConstraint* constraint)
{
    return springJoint->anchorA;
}

spVector 
spSpringJointGetAnchorB(spConstraint* constraint)
{
    return springJoint->anchorB;
}

spVector 
spSpringJointGetWorldAnchorA(spConstraint* constraint)
{
    return spxTransform(constraint->bodyA->xf, springJoint->anchorA);
}

spVector 
spSpringJointGetWorldAnchorB(spConstraint* constraint)
{
    return spxTransform(constraint->bodyB->xf, springJoint->anchorB);
}

spFloat 
spSpringJointGetRestLength(spConstraint* constraint)
{
    return springJoint->restLength;
}

spFloat 
spSpringJointGetFrequency(spConstraint* constraint)
{
    return springJoint->frequency;
}

spFloat 
spSpringJointGetDamping(spConstraint* constraint)
{
    return springJoint->damping;
}

void 
spSpringJointSetAnchorA(spConstraint* constraint, spVector anchorA)
{
    springJoint->anchorA = anchorA;
}

void 
spSpringJointSetAnchorB(spConstraint* constraint, spVector anchorB)
{
    springJoint->anchorB = anchorB;
}

void 
spSpringJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA)
{
    springJoint->anchorA = spxTransform(constraint->bodyA->xf, anchorA);
}

void 
spSpringJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    springJoint->anchorB = spxTransform(constraint->bodyB->xf, anchorB);
}

void 
spSpringJointSetRestLength(spConstraint* constraint, spFloat restLength)
{
    springJoint->restLength = restLength;
}

void 
spSpringJointSetFrequency(spConstraint* constraint, spFloat frequency)
{
    springJoint->frequency = frequency;
}

void 
spSpringJointSetDamping(spConstraint* constraint, spFloat damping)
{
    springJoint->damping = damping;
}