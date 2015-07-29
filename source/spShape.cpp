
#include "spSegment.h"
#include "spPolygon.h"
#include "spCircle.h"
#include "spShape.h"
#include "spBody.h"

/// common collision filters and a default material
extern const spMask spCollideAll   = ~(spMask)0;
extern const spMask spCollideNone  =  (spMask)0;
const spFilter spFilterCollideNone = {  (spMask)0,  (spMask)0,  (spMask)0 };
const spFilter spFilterCollideAll  = { ~(spMask)0, ~(spMask)0, ~(spMask)0 };
const spMaterial spDefaultMaterial = { 0.6f, 0.2f };

spMaterial 
spMaterialConstruct(spFloat restitution, spFloat friction)
{
    spMaterial material;
    material.restitution = restitution;
    material.friction = friction;
    return material;
}

void 
spMassDataInit(spMassData* data, const spVector center, spFloat inertia, spFloat mass)
{
    NULLCHECK(data);
    data->inertia = inertia;
    data->mass = mass;
    data->com = center;
}

void spShapeInit(spShape* shape, spMassData* data, spBound* bound, spShapeType type)
{
    NULLCHECK(shape);
    NULLCHECK(data);
    NULLCHECK(bound);
    shape->mass_data = *data;
    shape->material = spDefaultMaterial;
    shape->bound = *bound;
    shape->type = type;
    shape->body = NULL;
    shape->next = NULL;
    shape->prev = NULL;
    shape->filter = spFilterCollideAll;
}

void 
spShapeFree(spShape* shape)
{
    if (shape->body)
    {
        spBodyRemoveShape(shape->body, shape);
    }

    /// TODO: add function pointers to free shapes to reduce includes in spShape.cpp

    /// shape is a poly
    if (shape->type == SP_SHAPE_POLYGON)
    {
        spPolygon* poly = spShapeCastPolygon(shape);
        spPolygonFree(&poly);
    }

    /// shape is a circle
    else if (shape->type == SP_SHAPE_CIRCLE)
    {
        spCircle* circle = spShapeCastCircle(shape);
        spCircleFree(&circle);
    }

    /// shape is a segment
    else
    {
        spSegment* segment = spShapeCastSegment(shape);
        spSegmentFree(&segment);
    }
}

spBool 
spShapeLessThan(const spShape* a, const spShape* b)
{
    NULLCHECK(a); 
    NULLCHECK(b);
    return a < b ? spTrue : spFalse;
}

spBool 
spShapeTestPoint(spShape* shape, spVector point)
{
    NULLCHECK(shape);
    switch (shape->type)
    {
    case SP_SHAPE_CIRCLE:
        return spCircleTestPoint((spCircle*) shape, point);
    case SP_SHAPE_POLYGON:
        return spPolygonTestPoint((spPolygon*) shape, point);
    case SP_SHAPE_SEGMENT:
        return spFalse;
    }
    return spFalse;
}

spBool 
spShapesCanCollide(spShape* a, spShape* b)
{
    NULLCHECK(a); NULLCHECK(b);
    if (a->filter.group == spCollideAll && b->filter.group == spCollideAll) return spTrue;
    if (a->filter.group == b->filter.group) return spFalse;
    if (a->filter.type & b->filter.collide) return spTrue;
    if (b->filter.type & a->filter.collide) return spTrue;

    return spFalse;
}

struct spCircle* 
spShapeCastCircle(const spShape* shape)
{
    NULLCHECK(shape);
    if (shape->type == SP_SHAPE_CIRCLE)
    {
        return (spCircle*)shape;
    }
    else
    {
        spWarning(spFalse, "the shape is not a circle!\n");
        return NULL;
    }
}

struct spPolygon* 
spShapeCastPolygon(const spShape* shape)
{
    NULLCHECK(shape);
    if (shape->type == SP_SHAPE_POLYGON)
    {
        return (spPolygon*)shape;
    }
    else
    {
        spWarning(spFalse, "the shape is not a polygon!\n");
        return NULL;
    }
}

struct spSegment* 
spShapeCastSegment(const spShape* shape)
{
    NULLCHECK(shape);
    if (shape->type == SP_SHAPE_SEGMENT)
    {
        return (spSegment*)shape;
    }
    else
    {
        spWarning(spFalse, "the shape is not a segment!\n");
        return NULL;
    }
}

spShapeType 
spShapeGetType(spShape* shape)
{
    NULLCHECK(shape);
    return shape->type;
}

spMassData 
spShapeGetMassData(spShape* shape)
{
    NULLCHECK(shape);
    return shape->mass_data;
}

spVector 
spShapeGetCOM(const spShape* shape)
{
    NULLCHECK(shape);
    return shape->mass_data.com;
}

spFloat 
spShapeGetMass(const spShape* shape)
{
    NULLCHECK(shape);
    return shape->mass_data.mass;
}

spFloat 
spShapeGetInertia(const spShape* shape)
{
    NULLCHECK(shape);
    return shape->mass_data.inertia;
}

spMaterial 
spShapeGetMaterial(spShape* shape)
{
    NULLCHECK(shape);
    return shape->material;
}

spFilter 
spShapeGetFilter(spShape* shape)
{
    NULLCHECK(shape);
    return shape->filter;
}

spBound 
spShapeGetBound(spShape* shape)
{
    NULLCHECK(shape);
    return shape->bound;
}

spBody* 
spShapeGetBody(spShape* shape)
{
    NULLCHECK(shape);
    return shape->body;
}

spVector 
spShapeLocalToWorldPoint(spShape* shape, spVector point)
{
    NULLCHECK(shape);
    return spMult(shape->body->xf, point);
}

spVector 
spShapeWorldToLocalPoint(spShape* shape, spVector point)
{
    NULLCHECK(shape);
    return spTMult(shape->body->xf, point);
}

spVector 
spShapeLocalToWorldVector(spShape* shape, spVector vector)
{
    NULLCHECK(shape);
    return spMult(shape->body->xf.q, vector);
}

spVector 
spShapeWorldToLocalVector(spShape* shape, spVector vector)
{
    NULLCHECK(shape);
    return spTMult(shape->body->xf.q, vector);
}

void 
spShapeSetMaterial(spShape* shape, spFloat restitution, spFloat friction)
{
    NULLCHECK(shape);
    shape->material = { restitution, friction };
}

void 
spShapeSetNewMaterial(spShape* shape, spMaterial material)
{
    NULLCHECK(shape);
    shape->material = material;
}

void 
spShapeSetFriction(spShape* shape, spFloat friction)
{
    NULLCHECK(shape);
    shape->material.friction = friction;
}

void 
spShapeSetRestitution(spShape* shape, spFloat restitution)
{
    NULLCHECK(shape);
    shape->material.restitution = restitution;
}

void 
spShapeSetFilter(spShape* shape, const spFilter filter)
{
    NULLCHECK(shape);
    shape->filter = filter;
}

spFilter 
spFilterConstruct(spGroup group, spMask type, spMask collide)
{
    spFilter filter = { group, type, collide };
    return filter;
}