
#include "spSpringJoint.h"
#include "spDebugDraw.h"
#include "spBody.h"

#define springJoint spConstraintCastSpringJoint(constraint)

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
    joint->beta = 0.0f;
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

void 
spSpringJointFree(spSpringJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

void 
spSpringJointPreStep(spSpringJoint* joint, const spFloat h)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// get the anchors in world space
    spVector pA = spMult(a->xf, joint->anchorA);
    spVector pB = spMult(b->xf, joint->anchorB);

    /// calculate relative velocity and normal
    joint->rA = spSub(pA, a->p);
    joint->rB = spSub(pB, b->p);
    joint->n = spSub(pB, pA);
    spFloat length = spLength(joint->n);
    joint->n = spMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    /// compute inverse mass
    spFloat rcnA = spCross(joint->rA, joint->n);
    spFloat rcnB = spCross(joint->rB, joint->n);
    spFloat iMass = a->mInv + a->iInv * rcnA * rcnA + b->mInv + b->iInv * rcnB * rcnB;
    spFloat mass = 1.0f / (iMass + SP_FLT_EPSILON);

    /// compute position constraint
    spFloat C = length - joint->restLength;

    /// compute spring params
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * mass * joint->damping * omega;
    spFloat k = mass * omega * omega;
    joint->gamma = 1.0f / (h * (c + h * k) + SP_FLT_EPSILON);
    joint->beta = C * h * k * joint->gamma;

    /// compute effective mass
    iMass += joint->gamma;
    joint->eMass = 1.0f / (iMass + SP_FLT_EPSILON);

    /// reset lagrange multiplier
    joint->lambdaAccum = 0.0f;
}

void 
spSpringJointSolve(spSpringJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute velocity constraint
    spVector rvA = spAdd(a->v, spCross(a->w, joint->rA));
    spVector rvB = spAdd(b->v, spCross(b->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spSub(rvB, rvA));

    /// compute lagrange multiplier
    spFloat lambda = -joint->eMass * (Cdot + joint->beta + joint->gamma * joint->lambdaAccum);
    joint->lambdaAccum += lambda;

    /// compute the impulse
    spVector impulseB = spMult(joint->n, lambda);
    spVector impulseA = spNegate(impulseB);

    /// apply the impulse
    spBodyApplyImpulse(a, joint->rA, impulseA);
    spBodyApplyImpulse(b, joint->rB, impulseB);

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
    return spMult(constraint->bodyA->xf, springJoint->anchorA);
}

spVector 
spSpringJointGetWorldAnchorB(spConstraint* constraint)
{
    return spMult(constraint->bodyB->xf, springJoint->anchorB);
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
    springJoint->anchorA = spMult(constraint->bodyA->xf, anchorA);
}

void 
spSpringJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB)
{
    springJoint->anchorB = spMult(constraint->bodyB->xf, anchorB);
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