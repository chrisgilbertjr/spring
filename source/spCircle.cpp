
#include "spDebugDraw.h"
#include "spCircle.h"
#include "spBody.h"

static spFloat
spCircleComputeInertia(spCircle* circle, spFloat mass)
{
    spAssert(circle != NULL, "the circle is null while computing mass data");

    /// http://en.wikipedia.org/wiki/List_of_moments_of_inertia
    /// inertia for a 'disk' = mass * radius * radius / 2.0f
    return mass * circle->radius * circle->radius * 0.5f * SP_DEG_TO_RAD;
}

static void 
spCircleComputeBound(spCircle* circle, spBound* bound)
{
    spAssert(circle != NULL, "the circle is null while computing mass data");

    spFloat radius = circle->radius;

    bound->center = circle->center;
    bound->radius = radius;
    bound->half_width = spVector(radius, radius);
}

static void 
spCircleComputeMassData(spCircle* circle, spMassData* data, spFloat mass)
{
    spAssert(circle != NULL, "the circle is null while computing mass data");
    spAssert(data != NULL, "mass data is null while computing mass data");

    spMassDataInit(data, circle->center, spCircleComputeInertia(circle, mass), mass);
}

void 
spCircleInit(spCircle* circle, spVector center, spFloat radius, spFloat mass)
{
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
    spCircleInit(circle, center, radius, mass);
    return (spShape*)circle;
}

void 
spCircleFree(spCircle** circle)
{
    spFree(circle);
}

spBool 
spCircleTestPoint(spCircle* circle, spVector point)
{
    spVector center = spCircleGetWorldCenter(circle);
    return spLength(spSub(point, center)) <= circle->radius;
}

spVector 
spCircleGetLocalCenter(spCircle* circle)
{
    return circle->center;
}

spVector 
spCircleGetWorldCenter(spCircle* circle)
{
    return spMult(circle->shape.body->xf, circle->center);
}

spFloat 
spCircleGetRadius(spCircle* circle)
{
    return circle->radius;
}