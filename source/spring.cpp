
#include "spring.h"


//spBody* 
//spCreateBody(spWorld* world, spBodyType type)
//{
//    spWorldIsSane(world);

//    spBody* body = spBodyNew(type);
//    spBodyAdd(body, world->body_list);
//    return body;
//}

//void 
//spDestroyBody(spWorld* world, spBody* body)
//{
//    spWorldIsSane(world);
//    spBodyIsSane(body);

//    spBodyRemove(body, world->body_list);
//    spBodyFree(body);
//}

//void 
//spAddCircle(spCircle* circle, spBody* body)
//{
//    spShape* base_shape_pointer = (spShape*)circle;
//    spShapeAdd(base_shape_pointer, body->shape_list);
//    spBodyComputeShapeMassData(body);
//}

//void 
//spRemoveCircle(spCircle* circle)
//{
//    spBody* body = circle->base_class.body;
//    spAssert(body != NULL, "body is NULL in spRemoveCircle!\n");
//    spShape* shape_list = body->shape_list;
//    spShape* base_shape_pointer = (spShape*)circle;
//    spShapeRemove(base_shape_pointer, shape_list);
//}

//spCircle* 
//spCreateCircle(const spCircleDef& def)
//{
//    return spCreateCircle(0, def);
//}

//spCircle* 
//spCreateCircle(
//    const spVector&   center, 
//    spFloat           radius, 
//    const spMaterial& material, 
//    spFloat           mass)
//{
//    spCircleDef def;
//    def.center = center;
//    def.radius = radius;
//    def.material = material;
//    def.mass = mass;

//    return spCreateCircle(0, def);
//}

//spCircle* 
//spCreateCircle(spBody* body, const spCircleDef& def)
//{
//    spBodyIsSane(body);
//    spCircleDefIsSane(def);

//    spCircle* circle = spCircleNew(body, def);
//    spAddCircle(circle, body);
//    return circle;
//}

//spCircle* 
//spCreateCircle(
//    spBody*           body, 
//    const spVector&   center, 
//    spFloat           radius, 
//    const spMaterial& material, 
//    spFloat           mass)
//{
//    spCircleDef def;
//    def.center = center;
//    def.radius = radius;
//    def.material = material;
//    def.mass = mass;

//    return spCreateCircle(body, def);
//}

//void 
//spDestroyCircle(spCircle* circle)
//{
//    spCircleIsSane(circle);

//    spCircleFree(circle);
//}

//void 
//spAddPolygon(spPolygon* poly, spBody* body)
//{
//    spShape* base_shape_pointer = (spShape*) poly;
//    spShapeAdd(base_shape_pointer, body->shape_list);
//    spBodyComputeShapeMassData(body);
//}

//void 
//spRemovePolygon(spPolygon* poly)
//{
//    spBody* body = poly->base_class.body;
//    spAssert(body != NULL, "body is NULL in spRemovePolygon!\n");
//    spShape* shape_list = body->shape_list;
//    spShape* base_shape_pointer = (spShape*)poly;
//    spShapeRemove(base_shape_pointer, shape_list);;
//}

//spPolygon* 
//spCreatePolygon(const spPolygonDef& def)
//{
//    return spCreatePolygon(0, def);
//}

//spPolygon* 
//spCreatePolygon(spVector* vertices, spInt count, const spMaterial& material, spFloat mass)
//{
//    spPolygonDef def;
//    def.vertices = vertices;
//    def.vertex_count = count;
//    def.material = material;
//    def.mass = mass;

//    return spCreatePolygon(0, def);
//}

//spPolygon* 
//spCreatePolygon(spBody* body, const spPolygonDef& def)
//{
//    spBodyIsSane(body);
//    spPolygonDefIsSane(def);

//    spPolygon* polygon = spPolygonNew(body, def);
//    spAddPolygon(polygon, body);
//    return polygon;
//}

//spPolygon* 
//spCreatePolygon(spBody* body, spVector* vertices, spInt count, const spMaterial& material, spFloat mass)
//{
//    spPolygonDef def;
//    def.vertices = vertices;
//    def.vertex_count = count;
//    def.material = material;
//    def.mass = mass;

//    return spCreatePolygon(body, def);
//}

//void 
//spDestroyPolygon(spPolygon* poly)
//{
//    spPolygonIsSane(poly);

//    spRemovePolygon(poly);
//    spPolygonFree(poly);
//}

