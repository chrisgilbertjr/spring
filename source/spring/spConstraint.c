
#include "spConstraint.h"
#include "spWorld.h"

/// baumgarte velocity bias coef
spFloat spBaumgarte = 0.2f;

void 
spConstraintInitFuncs(spConstraintFuncs* funcs, spFreeFunc free, spPreSolveFunc preSolve, spWarmStartFunc warmStart, spSolveFunc solve)
{
    funcs->free = free;
    funcs->preSolve = preSolve;
    funcs->warmStart = warmStart;
    funcs->solve = solve;
}

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
spConstraintFree(spConstraint** constraint)
{
    spConstraint* Constraint = *constraint;

    /// remove the constraint from the world
    spWorldRemoveConstraint(Constraint->world, constraint);

    /// free the constraint
    Constraint->funcs.free(constraint);
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