
#include "spSpringJoint.h"
#include "spDebugDraw.h"
#include "spBody.h"

spVector 
spSpringJointGetWorldAnchorA(spSpringJoint* joint)
{
    return spMult(joint->constraint.body_a->xf, joint->anchorA);
}

spVector 
spSpringJointGetWorldAnchorB(spSpringJoint* joint)
{
    return spMult(joint->constraint.body_b->xf, joint->anchorB);
}

void 
spSpringJointSetWorldAnchorA(spSpringJoint* joint, spVector anchorA)
{
    joint->anchorA = spTMult(joint->constraint.body_a->xf, anchorA);
}

void 
spSpringJointSetWorldAnchorB(spSpringJoint* joint, spVector anchorB)
{
    joint->anchorB = spTMult(joint->constraint.body_a->xf, anchorB);
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
    joint->beta = 0.0f;
}

spSpringJoint* 
spSpringJointAlloc()
{
    return (spSpringJoint*) spMalloc(sizeof(spSpringJoint));
}

spSpringJoint* 
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
    spSpringJointInit(spring, a, b, anchorA, anchorB, frequency, damping, restLength);
    return spring;
}

void 
spSpringJointFree(spSpringJoint** joint)
{
    /// free the memory and set the pointers to NULL to be safe
    free(*joint);
    *joint = NULL;
    joint = NULL;
}

void 
spSpringJointPreStep(spSpringJoint* joint, const spFloat h)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;
    spVector pA = spMult(bA->xf, joint->anchorA);
    spVector pB = spMult(bB->xf, joint->anchorB);

    /// calculate relative velocity and normal
    joint->rA = spSub(pA, bA->p);
    joint->rB = spSub(pB, bB->p);
    joint->n = spSub(pB, pA);
    spFloat length = spLength(joint->n);
    joint->n = spMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    /// compute inverse mass
    spFloat rcnA = spCross(joint->rA, joint->n);
    spFloat rcnB = spCross(joint->rB, joint->n);
    spFloat iMass = bA->mInv + bA->iInv * rcnA * rcnA + bB->mInv + bB->iInv * rcnB * rcnB;
    spFloat mass = 1.0f / (iMass + SP_FLT_EPSILON);

    /// compute position error
    spFloat C = length - joint->restLength;

    /// compute spring frequency
    spFloat omega = 2.0f * SP_PI * joint->frequency;

    /// compute spring damping and stiffness
    spFloat c = 2.0f * mass * joint->damping * omega;
    spFloat k = mass * omega * omega;

    /// compute beta and gamma for soft constraint
    joint->gamma = 1.0f / (h * (c + h * k) + SP_FLT_EPSILON);
    joint->beta = C * h * k * joint->gamma;

    /// compute effective mass
    iMass += joint->gamma;
    joint->eMass = 1.0f / (iMass + SP_FLT_EPSILON);

    joint->lambdaAccum = 0.0f;
}

void 
spSpringJointSolve(spSpringJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    /// compute velocity constraint
    spVector rvA = spAdd(bA->v, spCross(bA->w, joint->rA));
    spVector rvB = spAdd(bB->v, spCross(bB->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spSub(rvB, rvA));

    /// compute lagrange multiplier of the constraint
    /// make the constraint soft using a beta and gamma value
    spFloat lambda = -joint->eMass * (Cdot + joint->beta + joint->gamma * joint->lambdaAccum);
    joint->lambdaAccum += lambda;

    /// apply the impulse
    spVector impulse = spMult(joint->n, lambda);
    bA->v  = spSub(bA->v, spMult(impulse, bA->mInv));
    bB->v  = spAdd(bB->v, spMult(impulse, bB->mInv));
    bA->w -= bA->iInv * spCross(joint->rA, impulse);
    bB->w += bB->iInv * spCross(joint->rB, impulse);
}