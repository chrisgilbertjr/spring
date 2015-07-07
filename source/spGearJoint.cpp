
#include "spGearJoint.h"
#include "spBody.h"

void 
spGearJointInit(spGearJoint* joint, spBody* a, spBody* b, spFloat ratio, spFloat phase)
{
    NULLCHECK(joint); NULLCHECK(a); NULLCHECK(b);
    joint->constraint = spConstraintConstruct(a, b, SP_GEAR_JOINT);
    joint->lambdaAccum = 0.0f;
    joint->ratioInv = ratio ? 1.0f / ratio : 0.0f;
    joint->ratio = ratio;
    joint->eMass = 0.0f;
    joint->phase = phase;
    joint->bias = 0.0f;
}

spGearJoint* 
spGearJointAlloc()
{
    return (spGearJoint*) spMalloc(sizeof(spGearJoint));
}

spConstraint* 
spGearJointNew(spBody* a, spBody* b, spFloat ratio, spFloat phase)
{
    spGearJoint* joint = spGearJointAlloc();
    NULLCHECK(joint);
    spGearJointInit(joint, a, b, ratio, phase);
    return (spConstraint*) joint;
}

void 
spGearJointFree(spGearJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

void 
spGearJointApplyCachedImpulses(spGearJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the impulse, and apply it to the bodies
    spFloat impulse = joint->lambdaAccum;
    a->w -= impulse * a->iInv * joint->ratioInv;
    b->w += impulse * b->iInv;
}

void 
spGearJointPreSolve(spGearJoint* joint, const spFloat h)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the effective mass
    spFloat iMass = a->iInv*joint->ratioInv + b->iInv*joint->ratio;
    joint->eMass = iMass ? 1.0f / iMass : 0.0f;

    /// compute the position constraint, and baumgarte velocity bias
    spFloat C = a->a * joint->ratio - b->a - joint->phase;
    spFloat beta = 0.2f;
    joint->bias =  C * beta / h;

    /// reset the lagrange multipler
    joint->lambdaAccum = 0.0f;
}

void 
spGearJointSolve(spGearJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the velocity constraint, and solve for the lagrange multiplier
    spFloat Cdot = b->w * joint->ratio - a->w;
    spFloat lambda = (joint->bias - Cdot) * joint->eMass;
    spFloat lambdaOld = joint->lambdaAccum;
    joint->lambdaAccum += lambda;

    /// compute and apply the impulse
    spFloat impulse = joint->lambdaAccum - lambdaOld;
    a->w -= impulse * a->iInv * joint->ratioInv;
    b->w += impulse * b->iInv;
}

spBool 
spConstraintIsGearJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    return constraint->type == SP_GEAR_JOINT;
}

spGearJoint*
spConstraintCastGearJoint(spConstraint* constraint)
{
    if (spConstraintIsGearJoint(constraint))
    {
        return (spGearJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a gear joint\n");
        return NULL;
    }
}

spFloat 
spGearJointGetImpulse(spGearJoint* joint)
{
    return joint->lambdaAccum;
}

spFloat 
spGearJointGetImpulseA(spGearJoint* joint)
{
    spBody* a = joint->constraint.bodyA;
    return -joint->lambdaAccum * a->iInv * joint->ratioInv;
}

spFloat 
spGearJointGetImpulseB(spGearJoint* joint)
{
    spBody* b = joint->constraint.bodyB;
    return joint->lambdaAccum * b->iInv;
}

spFloat 
spGearGetRatio(spGearJoint* joint)
{
    return joint->ratio;
}

spFloat 
spGearGetPhase(spGearJoint* joint)
{
    return joint->phase;
}

void 
spGearSetRatio(spGearJoint* joint, spFloat ratio)
{
    joint->ratio    = ratio != 0.0f ?      ratio : 0.0f;
    joint->ratioInv = ratio != 0.0f ? 1.0f/ratio : 0.0f;
}

void 
spGearSetPhase(spGearJoint* joint, spFloat phase)
{
    joint->phase = phase;
}