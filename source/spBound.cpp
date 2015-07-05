
#include "spBound.h"

/// initialize the bound
void spBoundInit(spBound* bound, spVector center, spFloat radius)
{
    NULLCHECK(bound);
    bound->center = center;
    bound->half_width = spVector(radius, radius);
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
    return bound->half_width.x * 4.0f + bound->half_width.y * 4.0f;
}

spFloat spBoundBoxArea(spBound* bound)
{
    NULLCHECK(bound);
    return bound->half_width.x * 2.0f * bound->half_width.y * 2.0f;
}

spBool 
spBoundBoxOverlap(spBound* a, spBound* b, spTransform* xfA, spTransform* xfB)
{
    NULLCHECK(a); NULLCHECK(b); NULLCHECK(xfA); NULLCHECK(xfB);
    spVector cA = spMult(*xfA, a->center);
    spVector cB = spMult(*xfB, b->center);

    if (spAbs(cA.x - cB.x) > (a->half_width.x + b->half_width.x)) return spFalse;
    if (spAbs(cA.y - cB.y) > (a->half_width.y + b->half_width.y)) return spFalse;
    return spTrue;
}

spVector spBoundGetHalfWidth(spBound* bound)
{
    NULLCHECK(bound);
    return bound->half_width;
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