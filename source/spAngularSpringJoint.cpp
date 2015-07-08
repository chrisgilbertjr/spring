
#include "spAngularSpringJoint.h"
#include "spBody.h"

void 
spAngularSpringJointInit(spAngularSpringJoint* joint, spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle)
{
    joint->constraint = spConstraintConstruct(a, b, SP_ANGULAR_SPRING_JOINT);
    joint->lambdaAccum = 0.0f;
    joint->restAngle = restAngle;
    joint->frequency = frequency;
    joint->damping = damping;
    joint->eMass = 0.0f;
    joint->gamma = 0.0f;
    joint->beta = 0.0f;
    joint->inverse = inverse;
}

spAngularSpringJoint* 
spAngularSpringJointAlloc()
{
    return (spAngularSpringJoint*) spMalloc(sizeof(spAngularSpringJoint));
}

spAngularSpringJoint* 
spAngularSpringJointNew(spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle)
{
    spAngularSpringJoint* joint = spAngularSpringJointAlloc();
    spAngularSpringJointInit(joint, a, b, inverse, frequency, damping, restAngle);
    return joint;
}

void 
spAngularSpringJointFree(spAngularSpringJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

void 
spAngularSpringJointPreSolve(spAngularSpringJoint* joint, const spFloat h)
{
    spBody* bA = joint->constraint.bodyA;
    spBody* bB = joint->constraint.bodyB;

    spFloat iMass = bA->iInv + bB->iInv;
    spFloat mass = iMass != 0.0f ? 1.0f / iMass : 0.0f;
    spFloat C;
    if (joint->inverse)
    {
        C = 0.5f * (bA->a - bB->a) - joint->restAngle;
    }
    else
    {
        C = 0.5f * (bB->a + bA->a) + joint->restAngle;
    }
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * mass * joint->damping * omega;
    spFloat k = mass * omega * omega;
    joint->gamma = h * (c + h * k);
    joint->gamma = joint->gamma ? 1.0f / joint->gamma : 0.0f;
    joint->beta = C * h * k * joint->gamma;

    iMass += joint->gamma;
    joint->eMass = iMass ? 1.0f / iMass : 0.0f;

    joint->lambdaAccum = 0.0f;
}

void 
spAngularSpringJointSolve(spAngularSpringJoint* joint)
{
    spBody* bA = joint->constraint.bodyA;
    spBody* bB = joint->constraint.bodyB;
    if (joint->inverse)
    {
        spFloat Cdot = bA->w - bB->w;
        spFloat lambda = -joint->eMass * (Cdot + joint->beta + joint->gamma * joint->lambdaAccum);
        joint->lambdaAccum += lambda;
        bA->w += lambda * bA->iInv;
        bB->w -= lambda * bB->iInv;
    }
    else
    {
        spFloat Cdot = bB->w + bA->w;
        spFloat lambda = -joint->eMass * (Cdot + joint->beta + joint->gamma * joint->lambdaAccum);
        joint->lambdaAccum += lambda;
        bA->w +=  lambda * bA->iInv;
        bB->w -= -lambda * bB->iInv;
    }
}