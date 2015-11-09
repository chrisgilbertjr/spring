
#include "spGearJoint.h"
#include "spBody.h"

/// convenience macro for getters/setters
#define gearJoint spConstraintCastGearJoint(constraint)

static void 
Free(spGearJoint** joint)
{
    NULLCHECK(*joint);
    spFree(joint);
}

static void 
PreSolve(spGearJoint* joint, const spFloat h)
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
}

static void 
WarmStart(spGearJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// apply last timesteps impulse
    a->w -= joint->lambdaAccum * a->iInv * joint->ratioInv;
    b->w += joint->lambdaAccum * b->iInv;

    /// reset the lagrange multipler
    joint->lambdaAccum = 0.0f;
}

static void 
Solve(spGearJoint* joint)
{
    /// get the bodies
    spBody* a = joint->constraint.bodyA;
    spBody* b = joint->constraint.bodyB;

    /// compute the velocity constraint, and solve for the lagrange multiplier
    spFloat Cdot = b->w * joint->ratio - a->w;
    spFloat lambda = (joint->bias - Cdot) * joint->eMass;
    joint->lambdaAccum += lambda;

    /// compute and apply the impulse
    spFloat impulse = lambda;
    a->w -= impulse * a->iInv * joint->ratioInv;
    b->w += impulse * b->iInv;
}


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
    spConstraintInitFuncs(&joint->constraint.funcs, 
        (spFreeFunc)Free, 
        (spPreSolveFunc)PreSolve, 
        (spWarmStartFunc)WarmStart, 
        (spSolveFunc)Solve);
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
spGearJointGetImpulse(spConstraint* constraint)
{
    return gearJoint->lambdaAccum;
}

spFloat 
spGearGetRatio(spConstraint* constraint)
{
    return gearJoint->ratio;
}

spFloat 
spGearGetPhase(spConstraint* constraint)
{
    return gearJoint->phase;
}

void 
spGearSetRatio(spConstraint* constraint, spFloat ratio)
{
    gearJoint->ratio    = ratio != 0.0f ?      ratio : 0.0f;
    gearJoint->ratioInv = ratio != 0.0f ? 1.0f/ratio : 0.0f;
}

void 
spGearSetPhase(spConstraint* constraint, spFloat phase)
{
    gearJoint->phase = phase;
}