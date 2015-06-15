
#include "spSpringJoint.h"
#include "spBody.h"

void 
spSpringJointInit(
    spSpringJoint* joint, 
	spBody*        a, 
	spBody*        b, 
	spVector       anchorA, 
	spVector       anchorB, 
	spFloat        restLength, 
	spFloat        frequency, 
	spFloat        damping)
{
    joint->constraint = spConstraintConstruct(a, b, SP_SPRING_JOINT);
    joint->anchorA = anchorA;
    joint->anchorB = anchorB;
    joint->rA = spVectorZero();
    joint->rB = spVectorZero();
    joint->n = spVectorZero();
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
	spFloat        restLength, 
	spFloat        frequency, 
	spFloat        damping)
{
    spSpringJoint* spring = spSpringJointAlloc();
    spSpringJointInit(spring, a, b, anchorA, anchorB, restLength, frequency, damping);
    return spring;
}

void 
spSpringJointFree(spSpringJoint** joint)
{
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

    joint->rA = spSub(pA, bA->p);
    joint->rB = spSub(pB, bB->p);
    joint->n = spSub(pB, pA);
    spFloat length = spLength(joint->n);
    joint->n = spMult(joint->n, 1.0f / (length + SP_FLT_EPSILON));

    spFloat rcnA = spCross(joint->rA, joint->n);
    spFloat rcnB = spCross(joint->rB, joint->n);
    spFloat iMass = bA->m_inv + bA->i_inv * rcnA * rcnA + bB->m_inv + bB->i_inv * rcnB * rcnB;
    spFloat mass = 1.0f / (iMass + SP_FLT_EPSILON);
    spFloat C = length - joint->restLength;
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * mass * joint->damping * omega; /// damping
    spFloat k = mass * omega * omega; /// spring stiffness
    joint->gamma = 1.0f / (h * (h * k + c) + SP_FLT_EPSILON);
    joint->beta = C * h * k * joint->gamma;

    iMass += joint->gamma;
    joint->eMass = 1.0f / (iMass + SP_FLT_EPSILON);
}

void 
spSpringJointSolve(spSpringJoint* joint)
{
    spBody* bA = joint->constraint.body_a;
    spBody* bB = joint->constraint.body_b;

    spVector rvA = spAdd(bA->v, spCross(bA->w, joint->rA));
    spVector rvB = spAdd(bB->v, spCross(bB->w, joint->rB));
    spFloat Cdot = spDot(joint->n, spSub(rvB, rvA));

    //spFloat lambda = -joint->eMass * (Cdot + joint->beta + joint->gamma * joint->lambdaAccum);
    //joint->lambdaAccum += lambda;
    spFloat lambda = -Cdot * joint->eMass;
    spFloat jPrev = joint->lambdaAccum;
    joint->lambdaAccum = jPrev + lambda;
    spFloat  j = joint->lambdaAccum - jPrev;
    spVector P = spMult(joint->n, j);

    //spVector P = spMult(joint->n, lambda);
    bA->v  = spSub(bA->v, spMult(P, bA->m_inv));
    bB->v  = spAdd(bB->v, spMult(P, bB->m_inv));
    bA->w -= bA->i_inv * spCross(joint->rA, P);
    bB->w += bB->i_inv * spCross(joint->rB, P);
}