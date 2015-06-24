
#include <spring.h> 
#include "spApplication.h"

#define PRESSED(key) glfwGetKey(app->window, key) == GLFW_PRESS
#define RELEASED(key) glfwGetKey(app->window, key) == GLFW_RELEASE

spCircleDef  g_circle;
spPolygonDef g_box;

spCircleDef circDef(spFloat radius, spFloat mass, spFloat rest, spFloat friction)
{
    g_circle.radius = radius;
    g_circle.material = spMaterial(friction, rest);
    g_circle.mass = mass;

    return g_circle;
}

spPolygonDef boxDef(spVector size, spFloat mass, spFloat rest, spFloat friction)
{
    g_box.vertices[0] = spVector(-size.x,-size.y); 
    g_box.vertices[1] = spVector( size.x,-size.y);
    g_box.vertices[2] = spVector( size.x, size.y);
    g_box.vertices[3] = spVector(-size.x, size.y);
    g_box.material = spMaterial(friction, rest);
    g_box.mass = mass;

    return g_box;
}

void init_globals()
{
    g_circle.center = spVectorZero();
    g_circle.mass = 10.f;
    g_circle.material = spMaterial(0.5f, 0.3f);
    g_circle.radius = 5.0f;

    g_box.vertex_count = 4;
    g_box.vertices = (spVector*) spMalloc(sizeof(spVector) * 4);
    g_box.vertices[0] = spVector(-1.0f,-1.0f); 
    g_box.vertices[1] = spVector( 1.0f,-1.0f);
    g_box.vertices[2] = spVector( 1.0f, 1.0f);
    g_box.vertices[3] = spVector(-1.0f, 1.0f);
    g_box.material = spMaterial(0.5f, 0.3f);
    g_box.mass = 10.0f;
}

void delete_globals()
{
    free(g_box.vertices);
}

void init_test(spApplication* app)
{
    spBody* body;
    spShape* box;
    spFilter filter = spFilterCollideAll;


    body = spBodyNewStatic();
    box = spPolygonNew(body, boxDef(spVector(1000.0f, 5.0f), 25.0f, 0.2f, 0.8f));
    spShapeSetFilter(box, filter);
    spBodySetTransform(body, spVector(0.0f, -100.0f), 0.0f);
    spWorldAddBody(&app->world, body);
    spBodyAddShape(body, box);

    body = spBodyNewStatic();
    box = spPolygonNew(body, boxDef(spVector(5.0f, 100.0f), 25.0f, 0.2f, 0.8f));
    spShapeSetFilter(box, filter);
    spBodySetTransform(body, spVector(-200.0f, 0.0f), 0.0f);
    spWorldAddBody(&app->world, body);
    spBodyAddShape(body, box);

    body = spBodyNewStatic();
    box = spPolygonNew(body, boxDef(spVector(5.0f, 100.0f), 25.0f, 0.2f, 0.8f));
    spShapeSetFilter(box, filter);
    spBodySetTransform(body, spVector(200.0f, 0.0f), 0.0f);
    spWorldAddBody(&app->world, body);
    spBodyAddShape(body, box);



    spInt num = 12;
    spFloat w = 10.0f;
    spFloat h = 10.0f;
    for (spInt i = 0; i < 12; ++i)
    {
        spFloat o = w * 2.2f;
        spFloat p = (spFloat)i;

        spFloat offsetx = (-num* o * 0.5f) + p*o;

        body = spBodyNewDynamic();
    	box = spPolygonNew(body, boxDef(spVector(p+12.0f, p+15.f), 250.0f, 0.2f, 0.8f));
    	spShapeSetFilter(box, filter);
    	spBodySetTransform(body, spVector(offsetx, 80.0f), 0.0f);
    	spWorldAddBody(&app->world, body);
    	spBodyAddShape(body, box);
    }

    body = spBodyNewDynamic();
    box = spCircleNew(body, circDef(40.0f, 100.0f, 0.4f, 0.5f));
    spBodySetTransform(body, spVector(0.0f, 200.0f), 0.0f);
    spWorldAddBody(&app->world, body);
    spBodyAddShape(body, box);
}

spApplication* test()
{
    return spApplicationNew(
        "test app",
        spViewport(800, 800), spFrustumUniform(250), 
        spVector(0.f, -98.f), 10, 1.f/60.f, 
        init_test, default_loop, default_main_loop, NULL);
}

int main()
{
    init_globals();
    return run(test());
    delete_globals();
}