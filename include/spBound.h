
#ifndef SP_BOUND_H
#define SP_BOUND_H

#include "spMath.h"

/// @defgroup spBound spBound
/// @{

/// 2d axis aligned bounding box and circle
struct spBound
{
    spVector half_width;
    spVector center;
    spFloat radius;
};

/// sanity checks
#ifdef SP_DEBUG
 #define spBoundIsSane(bound) _spBoundIsSane(bound)

 /// bound sanity check
 inline void _spBoundIsSane(const spBound& bound)
 {
     spAssert(bound.half_width.x > SP_FLT_EPSILON && bound.half_width.y > SP_FLT_EPSILON, "invalid bound half widths!");
     spAssert(bound.radius >= SP_FLT_EPSILON, "invalid radius!");
 }
#else
 #define spBoundIsSane(bound) 
#endif

/// initialize the bound
inline void spBoundInit(spBound* bound, const spVector& center, spFloat radius)
{
    bound->center = center;
    bound->half_width = spVector(radius, radius);
    bound->radius = radius;

    spBoundIsSane(*bound);
}

/// 'faked' constructor for stack allocation
inline spBound _spBound()
{
    spBound bound;
    spBoundInit(&bound, spVector(0.0f, 0.0f), 0.0f);
    return bound;
}

/// get a bounds half width
inline spVector spBoundGetHalfWidth(const spBound* bound)
{
    return bound->half_width;
}

/// get a bounds center
inline spVector spBoundGetCenter(const spBound* bound)
{
    return bound->center;
}

/// get a bounds radius
inline spFloat spBoundGetRadius(const spBound* bound)
{
    return bound->radius;
}

/// set the bound
inline void spBoundSet(spBound* bound, const spVector& center, const spFloat x_size, const spFloat y_size)
{
    bound->center = center;
    bound->half_width = spMult(spVector(x_size, y_size), 0.5f);
    bound->radius = spLength(spSub(spAdd(bound->half_width, center), center));

    spBoundIsSane(*bound);
}

/// gets the perimeter of the bounding box
inline spFloat spBoundBoxPerimeter(const spBound& bound)
{
    spBoundIsSane(bound);

    return bound.half_width.x * 4.0f + bound.half_width.y * 4.0f;
}

/// gets the area of the bounding box
inline spFloat spBoundBoxArea(const spBound& bound)
{
    spBoundIsSane(bound);

    return bound.half_width.x * 2.0f * bound.half_width.y * 2.0f;
}

/// checks if two AABB overlap
inline spBool spBoundBoxOverlap(const spBound& a, const spBound& b, const spTransform& xfa, const spTransform& xfb)
{
    spBoundIsSane(a); 
    spBoundIsSane(b); 

    spVector ca_world = spMult(xfa, a.center);
    spVector cb_world = spMult(xfb, b.center);

    if (spAbs(ca_world.x - cb_world.x) > (a.half_width.x + b.half_width.x)) return spFalse;
    if (spAbs(ca_world.y - cb_world.y) > (a.half_width.y + b.half_width.y)) return spFalse;
    return spTrue;
}

/// @}

#endif