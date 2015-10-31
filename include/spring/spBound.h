
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
SPRING_API void spBoundInit(spBound* bound, spVector center, spFloat radius);

/// constructor for stack allocation
SPRING_API spBound spBoundConstruct(spVector center, spFloat radius);

/// gets the perimeter of the bounding box
SPRING_API spFloat spBoundBoxPerimeter(spBound* bound);

/// gets the area of the bounding box
SPRING_API spFloat spBoundBoxArea(spBound* bound);

/// checks if two AABB overlap
SPRING_API spBool spBoundBoxOverlap(spBound* a, spBound* b, spTransform* xfa, spTransform* xfb);

/// get the bound in world space
SPRING_API spBound spBoundGetWorldBound(spBound* bound, spTransform* xf);

/// get the bounds center in world space
SPRING_API spVector spBoundGetWorldCenter(spBound* bound, spTransform* xf);

/// get the bounds half width
SPRING_API spVector spBoundGetHalfWidth(spBound* bound);

/// get the bounds center
SPRING_API spVector spBoundGetCenter(spBound* bound);

/// get the bounds radius
SPRING_API spFloat spBoundGetRadius(spBound* bound);

/// set the bounds half widths
SPRING_API void spBoundSetHalfWidth(spBound* bound, spVector halfWidth);

/// set the bounds center
SPRING_API void spBoundSetCenter(spBound* bound, spVector center);

/// set the bounds radius
SPRING_API void spBoundSetRadius(spBound* bound, spFloat radius);

/// @}

#endif