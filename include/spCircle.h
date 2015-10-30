
#ifndef SP_CIRCLE_H
#define SP_CIRCLE_H

#include "spShape.h"

/// @defgroup spCircle spCircle
/// @{

/// a circle shape that can be attached to rigid bodies
/// a circle is represented by a center in local space and a radius
struct spCircle
{
    spShape  shape;  ///< the base shape class
    spVector center; ///< the center of the circle in local space
    spFloat  radius; ///< the radius of the circle
};

/// check if a shape is a circle
spBool spShapeIsCircle(spShape* shape);

/// initialize a circle with a center and radius
void spCircleInit(spCircle* circle, spVector center, spFloat radius, spFloat mass);

/// allocate space for the circle on the heap
spCircle* spCircleAlloc();

/// allocate, and init a circle on the heap
spShape* spCircleNew(spVector center, spFloat radius, spFloat mass);

/// free a circle from the heap
void spCircleFree(spShape** circle);

/// tests if a point is inside of the circle
spBool spCircleTestPoint(spCircle* circle, spVector point);

/// gets the center in the circles local space
spVector spCircleGetLocalCenter(spCircle* circle);

/// gets the center in world space
spVector spCircleGetWorldCenter(spCircle* circle);

/// gets the circles radius
spFloat spCircleGetRadius(spCircle* circle);

/// @}

#endif