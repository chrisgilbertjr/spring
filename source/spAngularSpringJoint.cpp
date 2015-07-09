
#include "spAngularSpringJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define angularSpringJoint spConstraintCastAngularSpringJoint(constraint)

void 
spAngularSpringJointInit(spAngularSpringJoint* joint, spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle)
{
    NULLCHECK(joint);
    joint->constraint = spConstraintConstruct(a, b, SP_ANGULAR_SPRING_JOINT);
    joint->lambdaAccum = 0.0f;
    joint->restAngle = restAngle;
    joint->frequency = frequency;
    joint->damping = damping;
    joint->eMass = 0.0f;
    joint->gamma = 0.0f;
    joint->bias = 0.0f;
    joint->inverse = inverse;
}

spAngularSpringJoint* 
spAngularSpringJointAlloc()
{
    return (spAngularSpringJoint*) spMalloc(sizeof(spAngularSpringJoint));
}

spConstraint* 
spAngularSpringJointNew(spBody* a, spBody* b, spBool inverse, spFloat frequency, spFloat damping, spFloat restAngle)
{
    spAngularSpringJoint* joint = spAngularSpringJointAlloc();
    spAngularSpringJointInit(joint, a, b, inverse, frequency, damping, restAngle);
    return (spConstraint*) joint;
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
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the position constriant
    spFloat iMass = a->iInv + b->iInv;
    spFloat mass = iMass != 0.0f ? 1.0f / iMass : 0.0f;
    spFloat C;
    if (joint->inverse)
    {
        C = 0.5f * (a->a - b->a) - joint->restAngle;
    }
    else
    {
        C = 0.5f * (a->a + b->a) + joint->restAngle;
    }

    /// compute the spring params and bias velocity
    spFloat omega = 2.0f * SP_PI * joint->frequency;
    spFloat c = 2.0f * mass * joint->damping * omega;
    spFloat k = mass * omega * omega;
    joint->gamma = h * (c + h * k);
    joint->gamma = joint->gamma ? 1.0f / joint->gamma : 0.0f;
    joint->bias = C * h * k * joint->gamma;

    /// compute the effective mass
    iMass += joint->gamma;
    joint->eMass = iMass ? 1.0f / iMass : 0.0f;
}

void 
spAngularSpringJointApplyCachedImpulse(spAngularSpringJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// apply last timesteps impulses
    if (joint->inverse)
    {
        a->w += joint->lambdaAccum * a->iInv;
        b->w -= joint->lambdaAccum * b->iInv;
    }
    else
    {
        a->w +=  joint->lambdaAccum * a->iInv;
        b->w -= -joint->lambdaAccum * b->iInv;
    }

    /// reset the lagrange multiplier
    joint->lambdaAccum = 0.0f;
}

void 
spAngularSpringJointSolve(spAngularSpringJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// check which way the joints should rotate
    if (joint->inverse)
    {
        /// compute the velocity constraint
        spFloat Cdot = a->w - b->w;

        /// compute the lagrange multiplier
        spFloat lambda = -joint->eMass * (Cdot + joint->bias + joint->gamma * joint->lambdaAccum);
        joint->lambdaAccum += lambda;

        /// apply the impulses
        a->w += lambda * a->iInv;
        b->w -= lambda * b->iInv;
    }
    else
    {
        /// compute the velocity constraint
        spFloat Cdot = b->w + a->w;

        /// compute the lagrange multiplier
        spFloat lambda = -joint->eMass * (Cdot + joint->bias + joint->gamma * joint->lambdaAccum);
        joint->lambdaAccum += lambda;

        /// apply the impulses
        a->w +=  lambda * a->iInv;
        b->w -= -lambda * b->iInv;
    }
}

spBool 
spConstraintIsAngularSpringJoint(spConstraint* constraint)
{
    return constraint->type == SP_ANGULAR_SPRING_JOINT;
}

spAngularSpringJoint* 
spConstraintCastAngularSpringJoint(spConstraint* constraint)
{
    if (spConstraintIsAngularSpringJoint(constraint))
    {
        return (spAngularSpringJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not an angular spring joint\n");
        return NULL;
    }
}

spFloat 
spAngularSpringJointGetRestAngle(spConstraint* constraint)
{
    return angularSpringJoint->restAngle;
}

spFloat 
spAngularSpringJointGetFrequency(spConstraint* constraint)
{
    return angularSpringJoint->frequency;
}

spFloat 
spAngularSpringJointGetDamping(spConstraint* constraint)
{
    return angularSpringJoint->damping;
}

spBool 
spAngularSpringJointGetInverse(spConstraint* constraint)
{
    return angularSpringJoint->inverse;
}

void 
spAngularSpringJointSetRestAngle(spConstraint* constraint, spFloat restAngle)
{
    angularSpringJoint->restAngle = restAngle;
}

void 
spAngularSpringJointSetFrequency(spConstraint* constraint, spFloat frequency)
{
    angularSpringJoint->frequency = frequency;
}

void 
spAngularSpringJointSetDamping(spConstraint* constraint, spFloat damping)
{
    angularSpringJoint->damping = damping;
}

void 
spAngularSpringJointSetInverse(spConstraint* constraint, spBool inverse)
{
    angularSpringJoint->inverse = inverse;
}