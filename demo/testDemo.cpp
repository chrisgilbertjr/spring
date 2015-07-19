
#include "spDemo.h"
#include "spCircle.h"
#include "spPolygon.h"

//spShape* shapeA = NULL;
//spBody* bodyA = NULL;
//spShape* shapeB = NULL;
//spBody* bodyB = NULL;

static void setup() 
{
//    spFloat w = 0.2f;
//    spFloat h = 0.2f;
//    spVector verts[4] = {{-w, -h}, {w, -h}, {w, h}, {-w, h}};
//    bodyA = spBodyNewStatic();
//    shapeA = spPolygonNew(verts, 4, 12.0f);
//    spBodyAddShape(bodyA, shapeA);
//    spWorldAddBody(&demo->world, bodyA);

//    bodyB = spBodyNewDynamic();
//    shapeB = spCircleNew({0.0f, 0.0f}, 0.2f, 2.0f);
//    spBodyAddShape(bodyB, shapeB);
//    spWorldAddBody(&demo->world, bodyB);
}

static void update(spFloat dt) 
{ 
//    //spWorldStep(&demo->world, dt);

//    spPolygon* poly = (spPolygon*)shapeA;
//    spEdge* e = poly->edges;
//    spVector verts[4];
//    verts[0] = e[0].vertex;
//    verts[1] = e[1].vertex;
//    verts[2] = e[2].vertex;
//    verts[3] = e[3].vertex;
//    spDrawPolygon(spBodyGetPosition(bodyA), 0.0f, verts, 4, spShapeGetCOM(shapeA), BLUE(), WHITE());
//    spDrawCircle(spBodyGetPosition(bodyB), 0.0f, 0.2f, RED(), WHITE());
}

static void destroy() 
{
}

spDemo* test = spDemoNew(setup, update, destroy);
