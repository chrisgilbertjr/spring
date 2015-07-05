
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
    spFloat restitution; ///< 'bounciness' of a shape
    spFloat friction;    ///< friction of a shape
};

/// collision filter
struct spFilter
{
    spGroup group;   ///< objects in similar groups will not collide with one another
    spMask  type;    ///< bitmask that describes the type of filter
    spMask  collide; ///< bitmask that describes which types to collide with
};

/// TODO:
extern const spMask spCollideAll;
extern const spMask spCollideNone;
extern const spFilter spFilterCollideNone;
extern const spFilter spFilterCollideAll;

/// used to create shapes
struct spShapeDef
{
    spBody* body;
    spShapeType type;
    const spMassData* mass_data;
    const spMaterial* material;
    const spBound* bound;
};

/// TODO: document this better
/// a shape describes convex objects and are the base for collision primitives.
/// shapes describe the physical portion of a rigid body, and multiple may be attached to a single body.
/// each shape has mass properties, a material, a bound, and its type.
/// each body contains a linked list of shapes.
/// shapes must be created, but are released when a body is destroyed.
/// only destroy a shape if you want to remove it from a rigid body, otherwise let the body manage the shape.
struct spShape
{
    spShapeType type;     ///< INTERNAL: type of shape
    spMassData mass_data; ///<         : mass data of the shape (center of mass, mass, inertia)
    spMaterial material;  ///<         : the shapes material (restitution, friction)
    spFilter filter;      ///<         : used for collision filtering
    spShape* next;        ///< INTERNAL: next shape in the doubly linked list
    spShape* prev;        ///< INTERNAL: previous shape in the doubly linked list
    spBound  bound;       ///< INTERNAL: bounding volume of the shape
    spBody*  body;        ///< INTERNAL: the body the shape is attached to
};

/// initialize mass data with a mass, inertia, and center of mass
void spMassDataInit(spMassData* data, const spVector& center, spFloat inertia, spFloat mass);

/// initialize a shape with mass properties, a material, and a type
void spShapeInit(spShape* shape, const spShapeDef& def);

void spShapeInit2(spShape* shape, spMassData* data, spBound* bound, spShapeType type);

/// TODO:
spVector spShapeGetCenter(const spShape* shape);

/// TODO:
spFloat spShapeGetMass(const spShape* shape);

/// TODO:
spFloat spShapeGetInertia(const spShape* shape);

/// TODO:
void spShapeSetFilter(spShape* shape, const spFilter filter);

/// sorts two shapes based on their pointer values
spBool spShapeLessThan(const spShape* a, const spShape* b);

/// add a shape to a linked list
void spShapeAdd(spShape* shape, spShape*& shapes);

/// remove a shape from a linked list
void spShapeRemove(spShape* shape, spShape* shapes);

/// TODO:
spBool spShapeTestPoint(spShape* shape, spVector point);

/// TODO:
spBool spShapesCanCollide(spShape* a, spShape* b);

/// TODO:
struct spCircle* spShapeCastCircle(const spShape* shape);

/// TODO:
struct spPolygon* spShapeCastPolygon(const spShape* shape);

/// TODO:
spFilter spFilterConstruct(spGroup group, spMask type, spMask collide);

/// 'faked' constructor for stack allocation
spMaterial _spMaterial(const spFloat friction, const spFloat restitution);

/// compute the mixed friction of two materials
spFloat spMaterialComputeFriction(const spMaterial* ma, const spMaterial* mb);

/// compute the mixed restitution of two materials
spFloat spMaterialComputeRestitution(const spMaterial* ma, const spMaterial* mb);

/// sanity checks
#ifdef SP_DEBUG
 #define spMassDataIsSane(mass) _spMassDataIsSane(mass)
 #define spMaterialIsSane(material) _spMaterialIsSane(material)
 #define spShapeIsSane(shape) _spShapeIsSane(shape)

 /// mass data sanity check
 inline void _spMassDataIsSane(const spMassData& data)
 {
     spAssert(data.mass >= 0.0f, "mass is < zero in sanity check");
     spAssert(data.inertia >= 0.0f, "inertia is < zero in sanity check");
 }

 /// material sanity check
 inline void _spMaterialIsSane(const spMaterial& material)
 {
     spAssert(material.friction >= 0.0f, "friction is negative in sanity check");
     spAssert(material.restitution >= 0.0f, "restitution is negative in sanity check");
 }

 /// shape sanity check
 inline void _spShapeIsSane(const spShape* shape)
 {
     spMassDataIsSane(shape->mass_data);
     spMaterialIsSane(shape->material);
     spBoundIsSane(shape->bound);
     //spAssert(shape->body != NULL, "the shapes body is NULL in its sanity check");
 }
#else
 #define spMassDataIsSane(mass)
 #define spMaterialIsSane(material)
 #define spShapeIsSane(shape)
#endif

/// @}

#endif