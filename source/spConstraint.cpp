
#include "spDistanceJoint.h"
#include "spRopeJoint.h"

void 
spConstraintInit(spConstraint* constraint, spBody* a, spBody* b, spConstraintType type)
{
    constraint->type = type;
    constraint->next = NULL;
    constraint->prev = NULL;
    constraint->body_a = a;
    constraint->body_b = b;
}

spConstraint
spConstraintConstruct(spBody* a, spBody* b, spConstraintType type)
{
    spConstraint constraint;
    spConstraintInit(&constraint, a, b, type);
    return constraint;
}

void 
spConstraintPreStep(spConstraint* constraint, const spFloat h)
{
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointPreStep((spDistanceJoint*)constraint, h);
        break;
    case SP_ROPE_JOINT:
        spRopeJointPreStep((spRopeJoint*) constraint, h);
        break;
    default:
        spAssert(false, "constraint type is not valid in prestep!\n");
    }
}

void 
spConstraintSolve(spConstraint* constraint)
{
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointSolve((spDistanceJoint*) constraint);
        break;
    case SP_ROPE_JOINT:
        spRopeJointSolve((spRopeJoint*) constraint);
        break;
    default:
        spAssert(false, "constraint type is not valid in solve!\n");
    }
}

void 
spConstraintStabilize(spConstraint* constraint)
{
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointStabilize((spDistanceJoint*) constraint);
        break;
    case SP_ROPE_JOINT:
        spRopeJointStabilize((spRopeJoint*) constraint);
        break;
    default:
        spAssert(false, "constraint type is not valid in stabilize!\n");
    }
}