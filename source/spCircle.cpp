
#include "spCircle.h"

void 
spCircleInit(spCircle* circle, spBody* body, const spCircleDef& def)
{
    spCircleDefIsSane(def);

    const spMaterial* material = &def.material;
    spShapeDef shape_def;
    spMassData mass_data;
    spBound    bound;

    circle->center = def.center;
    circle->radius = def.radius;

    spCircleComputeMassData(circle, &mass_data, def.mass);
    spCircleComputeBound(circle, &bound);

    shape_def.type = SP_SHAPE_CIRCLE;
    shape_def.mass_data = &mass_data;
    shape_def.material = material;
    shape_def.body = body;
    shape_def.bound = &bound;

    spShapeInit(&circle->base_class, shape_def);

    spCircleIsSane(circle);
}

spCircle* 
spCircleAlloc()
{
    return (spCircle*) spMalloc(sizeof(spCircle));
}

spCircle* 
spCircleNew(spBody* body, const spCircleDef& def)
{
    spCircle* circle = spCircleAlloc();
    spCircleInit(circle, body, def);
    return circle;
}

void 
spCircleFree(spCircle*& circle)
{
    spFree(circle);
}

spFloat
spCircleComputeInertia(spCircle* circle, spFloat mass)
{
    spAssert(circle != NULL, "the circle is null while computing mass data");

    /// http://en.wikipedia.org/wiki/List_of_moments_of_inertia
    /// inertia for a 'disk' = mass * radius * radius / 2.0f
    return mass * circle->radius * circle->radius * 0.5f;
}

void 
spCircleComputeBound(spCircle* circle, spBound* bound)
{
    spAssert(circle != NULL, "the circle is null while computing mass data");

    spFloat radius = circle->radius;

    bound->center = circle->center;
    bound->radius = radius;
    bound->half_width = spVector(radius, radius);

    spBoundIsSane(*bound);
}

void 
spCircleComputeMassData(spCircle* circle, spMassData* data, spFloat mass)
{
    spAssert(circle != NULL, "the circle is null while computing mass data");
    spAssert(data != NULL, "mass data is null while computing mass data");

    spMassDataInit(data, circle->center, spCircleComputeInertia(circle, mass), mass);
}