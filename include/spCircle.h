
#ifndef SP_CIRCLE_H
#define SP_CIRCLE_H

#include "spShape.h"

/// @defgroup spCircle spCircle
/// @{

/// used to create circle shapes
struct spCircleDef
{
    spMaterial material; ///< describes the objects friction and 'bounciness'
    spVector center;     ///< the center of the circle in local space
    spFloat radius;      ///< the radius of the circle
    spFloat mass;        ///< the mass of the circle
};

/// TODO: document polygon
struct spCircle
{
    spShape  base_class; ///< the base shape class
    spFloat  radius;     ///< the radius of the circle
    spVector center;     ///< the center of the circle in local space
};

/// initialize a circle with a center and radius
void spCircleInit(spCircle* circle, spBody* body, const spCircleDef& def);

/// allocate space for the circle on the heap
spCircle* spCircleAlloc();

/// allocate, and init a circle on the heap
spCircle* spCircleNew(spBody* body, const spCircleDef& def);

/// free a circle from the heap
void spCircleFree(spCircle*& circle);

/// compute the moment of inertia for a circle
spFloat spCircleComputeInertia(spCircle* circle, spFloat mass);

/// compute the bounding volume of a circle
void spCircleComputeBound(spCircle* circle, spBound* bound);

/// compute mass, inertia, and center of mass for the shape
void spCircleComputeMassData(spCircle* circle, spMassData* data, spFloat mass);

/// sanity checks
#ifdef SP_DEBUG
 #define spCircleDefIsSane(circle) _spCircleDefIsSane(circle)
 #define spCircleIsSane(circle) _spCircleIsSane(circle)

 /// circle sanity check
 inline void _spCircleIsSane(const spCircle* circle)
 {
     spAssert(circle != NULL, "the circle is null in sanity check");
     spAssert(circle->radius > 0.0f, "the circle's radius is negative in the sanity check");
     spShapeIsSane(&circle->base_class);
 }

 /// circle def sanity check
 inline void _spCircleDefIsSane(const spCircleDef& def)
 {
     spMaterialIsSane(def.material);
     spAssert(def.radius > 0.0f, "the circle defs radius is negative in the sanity check");
     spAssert(def.mass >= 0.0f, "the circle defs mass is negative in the sanity check");
 }
#else
 #define spCircleDefIsSane(circle)
 #define spCircleIsSane(circle)
#endif

/// @}

#endif