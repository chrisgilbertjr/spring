
#ifndef SP_COLLISION_H
#define SP_COLLISION_H

#include "spPolygon.h"
#include "spCircle.h"

/// @defgroup spCollision spCollision
/// @{

/// a point on the minkowski difference of two shapes
struct spMinkowskiPoint
{
    spVector a; /// a point on shape a in world coords
    spVector b; /// a point on shape b in world coords
    spVector v; /// vector b - a
};

/// collision input for collide functions
struct spCollisionInput
{
    const spShapeType  type_a;
    const spShapeType  type_b;
    const spShape*     shape_a;
    const spShape*     shape_b;
    const spTransform* transform_a;
    const spTransform* transform_b;
};

/// function pointer for collision functions
typedef spBool (*spCollisionFunc)(spContact*& contact, const spCollisionInput& data);

/// A collision matrix for selecting which collision function to use based on a shape type
///    
///            circle        polygon       chain        B
///         +--------------+-------------+--------------+
/// circle  | circles      | circle+poly | circle+chain |
///         +--------------+-------------+--------------+
/// polygon | poly+circle  | polys       | poly+chain   |
///         +--------------+-------------+--------------+
/// chain   | chain+circle | chain+poly  | NONE         |
/// A       +-------------------------------------------+
struct spCollisionMatrix
{
    spCollisionFunc CollideFunc[SP_SHAPE_COUNT][SP_SHAPE_COUNT];
};

/// 'faked' constructor for stack allocation
spMinkowskiPoint spMinkowskiPointConstruct(spVector a, spVector b);

/// 'faked' constructor for stack allocation
spCollisionInput _spCollisionInput(const spShape* sa, const spShape* sb, const spTransform* xfa, const spTransform* xfb);

/// 'faked' constructor for stack allocation
spCollisionMatrix _spCollisionMatrix();

/// query the collision function from the collision matrix based on shape types
spCollisionFunc spCollisionQueryFunc(const spCollisionMatrix& matrix, spShapeType type_a, spShapeType type_b);

/// collide a circle and a polygon
//spBool spCollideCirclePolygon(spContact*& contact, const spCollisionInput& data);

///// collide a polygon and a circle
//spBool spCollidePolygonCircle(spContact*& contact, const spCollisionInput& data);

///// collide a circle and a chain
//spBool spCollideCircleChain(spContact*& contact, const spCollisionInput& data);

///// collide a chain and a circle
//spBool spCollideChainCircle(spContact*& contact, const spCollisionInput& data);

///// collide a polygon and a chain
//spBool spCollidePolygonChain(spContact*& contact, const spCollisionInput& data);

///// collide a chain and a polygon
//spBool spCollideChainPolygon(spContact*& contact, const spCollisionInput& data);

///// collide two circles
//spBool spCollideCircles(spContact*& contact, const spCollisionInput& data);

///// collide two polygons
//spBool spCollidePolygons(spContact*& contact, const spCollisionInput& data);

/// @}

#endif