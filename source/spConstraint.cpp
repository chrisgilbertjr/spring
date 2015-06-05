
#include "spConstraint.h"
#include "spDistanceJoint.h"

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
    case SP_DISTANCE_CONSTRAINT:
        spDistanceJointPreStep((spDistanceJoint*)constraint, h);
        break;
    case SP_HINGE_CONSTRAINT:
        /// TODO:
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
    case SP_DISTANCE_CONSTRAINT:
        spDistanceJointSolve((spDistanceJoint*)constraint);
        break;
    case SP_HINGE_CONSTRAINT:
        /// TODO:
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
    case SP_DISTANCE_CONSTRAINT:
        spDistanceJointStabilize((spDistanceJoint*) constraint);
        break;
    case SP_HINGE_CONSTRAINT:
        /// TODO:
        break;
    default:
        spAssert(false, "constraint type is not valid in stabilize!\n");
    }
}