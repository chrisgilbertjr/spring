
#ifndef SP_SHAPE_H
#define SP_SHAPE_H

#include "spLinkedList.h"
#include "spBound.h"

/// @defgroup spShape spShape
/// @{

/// different types of shapes
enum spShapeType
{
    SP_SHAPE_CIRCLE  = 0,
    SP_SHAPE_POLYGON = 1,
    SP_SHAPE_SEGMENT = 2,
    SP_SHAPE_COUNT
};

/// describes a shapes mass, inertia, and center of mass
struct spMassData
{
    spFloat  inertia; ///< moment of inertia
    spFloat  mass;    ///< mass
    spVector com;     ///< center of mass
};

/// describes the friction and bounciness of a shape
struct spMaterial
{
    spFloat restitution; ///< bounciness of a shape
    spFloat friction;    ///< friction of a shape
};

/// collision filtering via bitmasks/groups
struct spFilter
{
    spGroup group;   ///< objects in similar groups will not collide with one another
    spMask  type;    ///< bitmask that describes the type of filter
    spMask  collide; ///< bitmask that describes which types to collide with
};

/// shapes define geometry that will collide with other geometry
/// shapes need to be attached to rigid bodies, and added to the world
struct spShape
{
    spShapeType type;     ///< type of shape
    spMassData mass_data; ///< mass data of the shape (center of mass, mass, inertia)
    spMaterial material;  ///< the shapes material (restitution, friction)
    spFilter filter;      ///< used for collision filtering
    spShape* next;        ///< next shape in the doubly linked list
    spShape* prev;        ///< previous shape in the doubly linked list
    spBound  bound;       ///< bounding volume of the shape
    spBody*  body;        ///< the body the shape is attached to
};

/// create a physics material on the stack
spMaterial spMaterialConstruct(spFloat restitution, spFloat friction);

/// initialize mass data with a mass, inertia, and center of mass
void spMassDataInit(spMassData* data, const spVector center, spFloat inertia, spFloat mass);

/// initializes a shape with mass data, a bounding box, and a shape type
void spShapeInit(spShape* shape, spMassData* data, spBound* bound, spShapeType type);

/// free a shape from the heap
void spShapeFree(spShape* shape);

/// sorts two shapes based on their pointer values
spBool spShapeLessThan(const spShape* a, const spShape* b);

/// tests a point to see if it is inside of a shape
spBool spShapeTestPoint(spShape* shape, spVector point);

/// check if two shapes can collide via collision filtering
spBool spShapesCanCollide(spShape* a, spShape* b);

/// casts a shape into a circle pointer (if it is a circle shape)
struct spCircle* spShapeCastCircle(const spShape* shape);

/// casts a shape into a polygon pointer (if it is a polygon shape)
struct spPolygon* spShapeCastPolygon(const spShape* shape);

/// casts a shape into a segment pointer (if it is a segment shape)
struct spSegment* spShapeCastSegment(const spShape* shape);

/// get the shapes type
spShapeType spShapeGetType(spShape* shape);

/// get the shapes mass data
spMassData spShapeGetMassData(spShape* shape);

/// get the shapes center of mass
spVector spShapeGetCOM(const spShape* shape);

/// get the shapes mass
spFloat spShapeGetMass(const spShape* shape);

/// get the shapes moment of inertia
spFloat spShapeGetInertia(const spShape* shape);

/// get the shapes material
spMaterial spShapeGetMaterial(spShape* shape);

/// get the shapes collision filter
spFilter spShapeGetFilter(spShape* shape);

/// get the shapes bound
spBound spShapeGetBound(spShape* shape);

/// get the shapes body
spBody* spShapeGetBody(spShape* shape);

/// get a local point in world space
spVector spShapeLocalToWorldPoint(spShape* shape, spVector point);

/// get a world point in local space
spVector spShapeWorldToLocalPoint(spShape* shape, spVector point);

/// get a local point in world space
spVector spShapeLocalToWorldVector(spShape* shape, spVector vector);

/// get a world point in local space
spVector spShapeWorldToLocalVector(spShape* shape, spVector vector);

/// set the shapes material given restitution and a friction value
void spShapeSetMaterial(spShape* shape, spFloat restitution, spFloat friction);

/// set the shapes material given a new material
void spShapeSetNewMaterial(spShape* shape, spMaterial material);

/// set the shapes material friction
void spShapeSetFriction(spShape* shape, spFloat friction);

/// set the shapes material restitution
void spShapeSetRestitution(spShape* shape, spFloat restitution);

/// set the shapes collision filter
void spShapeSetFilter(spShape* shape, const spFilter filter);

/// creates a new collision filter on the stack
spFilter spFilterConstruct(spGroup group, spMask type, spMask collide);

/// common collision filters and a default material
extern const spMask spCollideAll;
extern const spMask spCollideNone;
extern const spFilter spFilterCollideNone;
extern const spFilter spFilterCollideAll; 
extern const spMaterial spDefaultMaterial;

/// @}

#endif