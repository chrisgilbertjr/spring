
#include "spDemo.h"
#include "spCircle.h"
#include "spPolygon.h"

spShape* shapeA = NULL;
spBody* bodyA = NULL;
spShape* shapeB = NULL;
spBody* bodyB = NULL;

static void setup() 
{
    spBody* body;
    spShape* box;
    spFilter filter = spFilterCollideAll;

    spVector vertices[4];
    spVector size = spVectorConstruct(100.0f, 25.0f);
    spFloat mass = 25.0f;
    vertices[0] = spVectorConstruct(-size.x,-size.y); 
    vertices[1] = spVectorConstruct( size.x,-size.y);
    vertices[2] = spVectorConstruct( size.x, size.y);
    vertices[3] = spVectorConstruct(-size.x, size.y);

    body = spBodyNewStatic();
    box = spPolygonNew(vertices, 4, 0.0f);
    box->material.restitution = 0.2f;
    box->material.friction = 0.8f;
    spShapeSetFilter(box, filter);
    spBodySetTransform(body, spVectorConstruct(0.0f, -100.0f), 0.0f);
    spBodyAddShape(body, box);
    spWorldAddBody(&demo->world, body);

    //body = spBodyNewStatic();
    //box = spSegmentNew(spVectorConstruct(-10.0f, 0.0f), spVectorConstruct(10.0f, 0.0f), 5.0f, 10.0f);
    //box->body = body;
    //spBodySetTransform(body, spVectorConstruct(150.0f, 50.0f), 0.0f);
    //spBodyAddShape(body, box);
    //spWorldAddBody(&demo->world, body);

    body = spBodyNewDynamic();
    box = spCircleNew(spVectorConstruct(0.0f, 0.0f), 20.0f, 5.0f);
    box->material.restitution = 0.4f;
    box->material.friction = 0.5f;
    spBodySetTransform(body, spVectorConstruct(-50.0f, 0.0f), 0.0f);
    spBodyAddShape(body, box);
    spWorldAddBody(&demo->world, body);

    body = spBodyNewDynamic();
    box = spCircleNew(spVectorConstruct(0.0f, 0.0f), 11.0f, 8.0f);
    box->material.restitution = 0.4f;
    box->material.friction = 0.5f;
    spBodySetTransform(body, spVectorConstruct(-45.0f, 50.0f), 0.0f);
    spBodyAddShape(body, box);
    spWorldAddBody(&demo->world, body);

    //body = spBodyNewStatic();
    //box = spSegmentNew(spVectorConstruct(-20.0f, 0.0f), spVectorConstruct(20.0f, 0.0f), 10.0f, 10.0f);
    //box->material.restitution = 0.2f;
    //box->material.friction = 0.8f;
    //spBodySetTransform(body, spVectorConstruct(0.0f, 50.0f), 0.0f);
    //spBodyAddShape(body, box);
    //spWorldAddBody(&demo->world, body);
}

spFloat gy = 0.0f;

static void update(spFloat dt) 
{ 
    spWorldStep(&demo->world, dt);

    spBody* bodies = spWorldGetBodyList(&demo->world);
    while (bodies)
    {
        spBody* body = bodies;
        spShape* shapes = spBodyGetShapeList(body);
        while(shapes)
        {
            spShape* shape = shapes;
            spDemoDrawShape(shape);
            shapes = shape->next;
        }
        bodies = spBodyGetNext(body);
    }
    spFloat s = 5.0f;
    spVector vertices[4] = {{-s, -s}, {s, -s}, {s, s+5.0f}, {-s, s}};
    spVector center = {0.0f, 0.0f};
    spDrawPolygon({0.0f, 0.0f}, 0.0f, vertices, 4, center, GREEN(), WHITE());

    spDrawLine({-10.0f, 0.0f}, {10.0f, gy}, 1.0f);
    gy += 0.1f;
}

static void destroy() 
{
}

spDemo* test = spDemoNew(setup, update, destroy);
