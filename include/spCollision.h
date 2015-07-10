
#ifndef SP_COLLISION_H
#define SP_COLLISION_H

/// @defgroup spCollision spCollision
/// @{

#include "spShape.h"

/// tells you information on two shapes contact information, returned by collision functions
struct spCollisionResult
{
    spBool   colliding; ///< spTrue if the the shapes are in contact, spFlase otherwise
    spVector normal;    ///< collision normal
    spVector pointA[2]; ///< collision points, for body A
    spVector pointB[2]; ///< collision points, for body B
    spInt    count;     ///< number of contact points
};

/// collision/support function pointer typedefs
typedef struct spCollisionResult (*spCollisionFunc)(const struct spShape* shapeA, const struct spShape* shapeB);
typedef struct spVector (*SupportPointFunc)(const struct spShape* shapeA, const spVector normal);

/// list of collision functions, indexed by the shapes types
extern spCollisionFunc CollideFunc[SP_SHAPE_COUNT][SP_SHAPE_COUNT];

/// @}

#endif