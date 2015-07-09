
#ifndef SP_BOUND_H
#define SP_BOUND_H

#include "spMath.h"

/// @defgroup spBound spBound
/// @{

/// 2d axis aligned bounding box and circle
struct spBound
{
    spVector halfWidth; ///< half width of bounding box
    spVector center;    ///< center of circle
    spFloat  radius;    ///< radius of circle
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

/// get the bound in world space
spBound spBoundGetWorldBound(spBound* bound, spTransform* xf);

/// get the bounds center in world space
spVector spBoundGetWorldCenter(spBound* bound, spTransform* xf);

/// get the bounds half width
spVector spBoundGetHalfWidth(spBound* bound);

/// get the bounds center
spVector spBoundGetCenter(spBound* bound);

/// get the bounds radius
spFloat spBoundGetRadius(spBound* bound);

/// set the bounds half widths
void spBoundSetHalfWidth(spBound* bound, spVector halfWidth);

/// set the bounds center
void spBoundSetCenter(spBound* bound, spVector center);

/// set the bounds radius
void spBoundSetRadius(spBound* bound, spFloat radius);

/// @}

#endif