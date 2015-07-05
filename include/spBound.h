
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
    spFloat  radius;
};

/// initialize the bound
void spBoundInit(spBound* bound, spVector center, spFloat radius);

/// constructor for stack allocation
spBound spBoundConstruct(spVector center, spFloat radius);

/// gets the perimeter of the bounding box
spFloat spBoundBoxPerimeter(spBound* bound);

/// gets the area of the bounding box
spFloat spBoundBoxArea(spBound* bound);

/// checks if two AABB overlap
spBool spBoundBoxOverlap(spBound* a, spBound* b, spTransform* xfa, spTransform* xfb);

/// get a bounds half width
spVector spBoundGetHalfWidth(spBound* bound);

/// get a bounds center
spVector spBoundGetCenter(spBound* bound);

/// get a bounds radius
spFloat spBoundGetRadius(spBound* bound);

/// @}

#endif