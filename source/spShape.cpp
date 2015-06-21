
#include "spPolygon.h"
#include "spCircle.h"
#include "spShape.h"

void 
spMassDataInit(spMassData* data, const spVector& center, spFloat inertia, spFloat mass)
{
    data->inertia = inertia;
    data->mass = mass;
    data->com = center;

    spMassDataIsSane(*data);
}

void 
spShapeInit(spShape* shape, const spShapeDef& def)
{
    shape->mass_data = *def.mass_data;
    shape->material = *def.material;
    shape->bound = *def.bound;
    shape->body = def.body;
    shape->type = def.type;
    shape->next = NULL;
    shape->prev = NULL;

    spShapeIsSane(shape);
}

spBool 
spShapeLessThan(const spShape* a, const spShape* b)
{
    return a < b ? spTrue : spFalse;
}

void 
spShapeAdd(spShape* shape, spShape*& shape_list)
{
    SP_LINKED_LIST_PREPEND(spShape, shape, shape_list);
    int x = 0;
}

void 
spShapeRemove(spShape* shape, spShape* shape_list)
{
    SP_LINKED_LIST_REMOVE(spShape, shape, shape_list);
}

spBool 
spShapeTestPoint(spShape* shape, spVector point)
{
    switch (shape->type)
    {
    case SP_SHAPE_CIRCLE:
        return spCircleTestPoint((spCircle*) shape, point);
    case SP_SHAPE_POLYGON:
        return spPolygonTestPoint((spPolygon*) shape, point);
    }
    return spFalse;
}

spMaterial 
_spMaterial(const spFloat friction, const spFloat restitution)
{
    spMaterial material;
    material.friction = friction;
    material.restitution = restitution;
    spMaterialIsSane(material);
    return material;
}

spFloat 
spMaterialComputeFriction(const spMaterial* ma, const spMaterial* mb)
{
    return spsqrt(ma->friction * mb->friction);
}

spFloat 
spMaterialComputeRestitution(const spMaterial* ma, const spMaterial* mb)
{
    return spMax(ma->restitution, mb->restitution);
}
