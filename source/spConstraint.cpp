
#include "spAngularSpringJoint.h"
#include "spDistanceJoint.h"
#include "spSpringJoint.h"
#include "spPointJoint.h"
#include "spMotorJoint.h"
#include "spWheelJoint.h"
#include "spMouseJoint.h"
#include "spRopeJoint.h"
#include "spGearJoint.h"
#include "spWorld.h"

spFloat spBaumgarte = 0.2f;

void 
spConstraintInit(spConstraint* constraint, spBody* a, spBody* b, spConstraintType type)
{
    NULLCHECK(constraint);
    constraint->type = type;
    constraint->next = NULL;
    constraint->prev = NULL;
    constraint->bodyA = a;
    constraint->bodyB = b;
    constraint->world = NULL;
}

spConstraint
spConstraintConstruct(spBody* a, spBody* b, spConstraintType type)
{
    spConstraint constraint;
    spConstraintInit(&constraint, a, b, type);
    return constraint;
}

void 
spConstraintFree(spConstraint* constraint)
{
    /// remove the constraint from the world
    spWorldRemoveConstraint(constraint->world, constraint);

    /// free the constraint
    constraint->funcs.free(constraint);
}

spConstraint* 
spConstraintGetNext(spConstraint* constraint)
{
    return constraint->next;
}

spConstraint* 
spConstraintGetPrev(spConstraint* constraint)
{
    return constraint->prev;
}

struct spBody* 
spConstraintGetBodyA(spConstraint* constraint)
{
    return constraint->bodyA;
}

struct spBody* 
spConstraintGetBodyB(spConstraint* constraint)
{
    return constraint->bodyB;
}

spConstraintType 
spConstraintGetType(spConstraint* constraint)
{
    return constraint->type;
}