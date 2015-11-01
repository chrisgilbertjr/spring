
#include "spBound.h"

/// initialize the bound
void spBoundInit(spBound* bound, spVector center, spFloat radius)
{
    NULLCHECK(bound);
    bound->center = center;
    bound->halfWidth = spVectorConstruct(radius, radius);
    bound->radius = radius;
}

spBound spBoundConstruct(spVector center, spFloat radius)
{
    spBound bound;
    spBoundInit(&bound, center, radius);
    return bound;
}

spFloat spBoundBoxPerimeter(spBound* bound)
{
    NULLCHECK(bound);
    return bound->halfWidth.x * 4.0f + bound->halfWidth.y * 4.0f;
}

spFloat spBoundBoxArea(spBound* bound)
{
    NULLCHECK(bound);
    return bound->halfWidth.x * 2.0f * bound->halfWidth.y * 2.0f;
}

spBool 
spBoundBoxOverlap(spBound* a, spBound* b, spTransform* xfA, spTransform* xfB)
{
    NULLCHECK(a); NULLCHECK(b); NULLCHECK(xfA); NULLCHECK(xfB);
    spVector cA = spMultXformVec(*xfA, a->center);
    spVector cB = spMultXformVec(*xfB, b->center);

    if (spAbs(cA.x - cB.x) > (a->halfWidth.x + b->halfWidth.x)) return spFalse;
    if (spAbs(cA.y - cB.y) > (a->halfWidth.y + b->halfWidth.y)) return spFalse;
    return spTrue;
}

spBound 
spBoundGetWorldBound(spBound* bound, spTransform* xf)
{
    NULLCHECK(bound); NULLCHECK(xf);
    spBound newBound;
    newBound.center = spMultXformVec(*xf, bound->center);
    newBound.halfWidth = bound->halfWidth;
    newBound.radius = bound->radius;
    return newBound;
}

spVector 
spBoundGetWorldCenter(spBound* bound, spTransform* xf)
{
    NULLCHECK(bound); NULLCHECK(xf);
    return spMultXformVec(*xf, bound->center);
}

spVector spBoundGetHalfWidth(spBound* bound)
{
    NULLCHECK(bound);
    return bound->halfWidth;
}

spVector 
spBoundGetCenter(spBound* bound)
{
    NULLCHECK(bound);
    return bound->center;
}

spFloat 
spBoundGetRadius(spBound* bound)
{
    NULLCHECK(bound);
    return bound->radius;
}

void 
spBoundSetHalfWidth(spBound* bound, spVector halfWidth)
{
    NULLCHECK(bound);
    bound->halfWidth = halfWidth;
}

void 
spBoundSetCenter(spBound* bound, spVector center)
{
    NULLCHECK(bound);
    bound->center = center;
}

void 
spBoundSetRadius(spBound* bound, spFloat radius)
{
    NULLCHECK(bound);
    bound->radius = radius;
    bound->halfWidth = spVectorConstruct(radius, radius);
}