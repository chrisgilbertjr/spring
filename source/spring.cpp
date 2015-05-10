
#include "spring.h"


spBody* 
spCreateBody(spWorld* world, spBodyType type)
{
    spWorldIsSane(world);

    spBody* body = spBodyNew(type);
    spBodyAdd(body, world->body_list);
    return body;
}

void 
spDestroyBody(spWorld* world, spBody* body)
{
    spWorldIsSane(world);
    spBodyIsSane(body);

    spBodyRemove(body, world->body_list);
    spBodyFree(body);
}

spCircle* 
spCreateCircle(spBody* body, const spCircleDef& def)
{
    spBodyIsSane(body);
    spCircleDefIsSane(def);

    spCircle* circle = spCircleNew(body, def);
    spShape* base_shape_pointer = (spShape*)circle;
    spShapeAdd(base_shape_pointer, body->shape_list);
    spBodyComputeShapeMassData(body);
    return circle;
}

void 
spDestroyCircle(spCircle* circle)
{
    spCircleIsSane(circle);

    spShape* shape_list = circle->base_class.body->shape_list;
    spShape* base_shape_pointer = (spShape*)circle;
    spShapeRemove(base_shape_pointer, shape_list);
    spCircleFree(circle);
}

spPolygon* 
spCreatePolygon(spBody* body, const spPolygonDef& def)
{
    spBodyIsSane(body);
    spPolygonDefIsSane(def);

    spPolygon* polygon = spPolygonNew(body, def);
    spShape* base_shape_pointer = (spShape*) polygon;
    spShapeAdd(base_shape_pointer, body->shape_list);
    spBodyComputeShapeMassData(body);
    return polygon;
}

void 
spDestroyPolygon(spPolygon* poly)
{
    spPolygonIsSane(poly);

    spShape* shape_list = poly->base_class.body->shape_list;
    spShape* base_shape_pointer = (spShape*) poly;
    spShapeRemove(base_shape_pointer, shape_list);
    spPolygonFree(poly);
}

