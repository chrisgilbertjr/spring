
#include "spCircle.h"
#include "spBody.h"

static spFloat
spCircleComputeInertia(spCircle* circle, spFloat mass)
{
    /// http://en.wikipedia.org/wiki/List_of_moments_of_inertia
    /// inertia for a 'disk' = mass * radius * radius / 2.0f
    NULLCHECK(circle);
    return mass * circle->radius * circle->radius * 0.5f * SP_DEG_TO_RAD;
}

static void 
spCircleComputeBound(spCircle* circle, spBound* bound)
{
    NULLCHECK(circle); NULLCHECK(bound);
    spBoundInit(bound, circle->center, circle->radius);
}

static void 
spCircleComputeMassData(spCircle* circle, spMassData* data, spFloat mass)
{
    NULLCHECK(circle); NULLCHECK(data);
    spMassDataInit(data, circle->center, spCircleComputeInertia(circle, mass), mass);
}

spBool 
spShapeIsCircle(spShape* shape)
{
    return shape->type == SP_SHAPE_CIRCLE;
}

void 
spCircleInit(spCircle* circle, spVector center, spFloat radius, spFloat mass)
{
    NULLCHECK(circle); 
    circle->center = center;
    circle->radius = radius;

    spMassData mass_data;
    spBound    bound;

    spCircleComputeMassData(circle, &mass_data, mass);
    spCircleComputeBound(circle, &bound);

    spShapeInit(&circle->shape, &mass_data, &bound, SP_SHAPE_CIRCLE);
}

spCircle* 
spCircleAlloc()
{
    return (spCircle*) spMalloc(sizeof(spCircle));
}

spShape* 
spCircleNew(spVector center, spFloat radius, spFloat mass)
{
    spCircle* circle = spCircleAlloc();
    NULLCHECK(circle);
    spCircleInit(circle, center, radius, mass);
    return (spShape*)circle;
}

void 
spCircleFree(spShape** circle)
{
    NULLCHECK(circle);
    NULLCHECK(*circle);
    spAssert(spShapeIsCircle(*circle), "The shape being freed is not a circle!\n");
    spFree(circle);
}

spBool 
spCircleTestPoint(spCircle* circle, spVector point)
{
    NULLCHECK(circle);
    spVector center = spCircleGetWorldCenter(circle);
    return spvLength(spvSub(point, center)) <= circle->radius;
}

spVector 
spCircleGetLocalCenter(spCircle* circle)
{
    NULLCHECK(circle);
    return circle->center;
}

spVector 
spCircleGetWorldCenter(spCircle* circle)
{
    NULLCHECK(circle);
    return spShapeLocalToWorldPoint(&circle->shape, circle->center);
}

spFloat 
spCircleGetRadius(spCircle* circle)
{
    NULLCHECK(circle);
    return circle->radius;
}