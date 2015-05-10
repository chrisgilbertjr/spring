
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
    SP_SHAPE_CHAIN   = 2,
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
    spShapeType type;     ///< type of shape
    spMassData mass_data; ///< mass data of the shape (center of mass, mass, inertia)
    spMaterial material;  ///< the shapes material (restitution, friction)
    spShape* next;        ///< next shape in the doubly linked list
    spShape* prev;        ///< previous shape in the doubly linked list
    spBody*  body;        ///< the body the shape is attached to
    spBound  bound;       ///< bounding volume of the shape
};

/// initialize mass data with a mass, inertia, and center of mass
void spMassDataInit(spMassData* data, const spVector& center, spFloat inertia, spFloat mass);

/// initialize a shape with mass properties, a material, and a type
void spShapeInit(spShape* shape, const spShapeDef& def);

/// sorts two shapes based on their pointer values
spBool spShapeLessThan(const spShape* a, const spShape* b);

/// add a shape to a linked list
void spShapeAdd(spShape* shape, spShape*& shape_list);

/// remove a shape from a linked list
void spShapeRemove(spShape* shape, spShape* shape_list);

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
     spAssert(shape->body != NULL, "the shapes body is NULL in its sanity check");
 }
#else
 #define spMassDataIsSane(mass)
 #define spMaterialIsSane(material)
 #define spShapeIsSane(shape)
#endif

/// @}

#endif