
#include <spring.h> 
#include "spApplication.h"

void create_square(
    spBody* new_body,
    spPolygon* new_poly,
    spWorld* wp,
    spFloat x,
    spFloat y,
    spFloat friction,
    spFloat restitution,
    spFloat g_scale,
    spFloat mass,
    spFloat size)
{
    new_body = spCreateBody(wp, SP_BODY_DYNAMIC);

    spVector verts[4];
    verts[0].x = -size;  verts[0].y = -size;
    verts[1].x =  size;  verts[1].y = -size;
    verts[2].x =  size;  verts[2].y =  size;
    verts[3].x = -size;  verts[3].y =  size;

    spPolygonDef pdd;
    pdd.mass = mass;
    pdd.material.friction = friction;
    pdd.material.restitution = restitution;
    pdd.vertex_count = 4;
    pdd.vertices = verts;

    new_poly = spCreatePolygon(new_body, pdd);

    spBodySetPosition(new_body, spVector(x, y));
    new_body->g_scale = g_scale;
}

void create_circle(
    spBody* new_body, 
    spCircle* new_circle, 
    spWorld* wp, 
    spFloat x,
    spFloat y,
    spFloat friction,
    spFloat restitution,
    spFloat g_scale,
    spFloat mass,
    spFloat radius)
{
    new_body = spCreateBody(wp, SP_BODY_DYNAMIC);

    spCircleDef cd;
    cd.mass = mass;
    cd.material.friction = friction;
    cd.material.restitution = restitution;
    cd.center = spVectorZero();
    cd.radius = radius;

    new_circle = spCreateCircle(new_body, cd);

    spBodySetPosition(new_body, spVector(x, y));
    new_body->g_scale = g_scale;
}

spApplication* test_application()
{
    return spApplicationNew(
        "test application",
        spViewport(800, 800), spFrustumUniform(50.0f),
        spVector(0.0f, -1.0f),
        20, 1.0f / 60.0f,
        default_init, default_loop, default_main_loop,
        0);
}

int main()
{
    run(test_application());
    //SP_RUN_TESTS(bound_tests);
    //SP_RUN_TESTS(chain_tests);
    //SP_RUN_TESTS(circle_tests);
    //SP_RUN_TESTS(math_tests);
    //SP_RUN_TESTS(polygon_tests);
    //SP_RUN_TESTS(shape_tests);

    //GLFWwindow* window;

    //if (!glfwInit())
    //{
    //    return -1;
    //}

    //window = glfwCreateWindow(800, 800, "Test Application", 0, 0);
    
    //if (!window)
    //{
    //    glfwTerminate();
    //    return -1;
    //}

    //glfwMakeContextCurrent(window);

    //spWorld world = spWorld(spVector(0.0f, -20.0f));
    //spWorld* wp = &world;
    //wp->iterations = 20;

    //const spInt SIZE = 64;
    //spBody bodies[SIZE];
    //spCircle circles[SIZE];

    //spFloat friction = 1.0f;
    //spFloat restitution = 0.7f;

    ////spBody* sb = 0;
    ////spPolygon* poly = 0;
    ////create_square(sb, poly, wp, -7.0f, 50.0f, 1.0f, 0.2f, 1.5f, 5.0f, 2.0f);

    ////for (spInt i = 0; i < 16; ++i)
    ////{
    ////    spFloat f = (spFloat)i;
    ////    spFloat f2 = (spFloat)i * 10.0f - 100;
    ////    spFloat g_scale = spClamp(f/8.0f, 0.0f, 1.0f);
    ////    create_circle(bodies+i, circles+i, wp, f2, 25.0f, friction, restitution, g_scale, 2.0f, 0.8f);
    ////}

    ////for (spInt i = 16; i < 20; ++i)
    ////{
    ////    spFloat f = ((spFloat)i - 16.0f);
    ////    spFloat f2 = ((spFloat)i - 16.0f) * 25.0f - 37.5f;
    ////    create_circle(bodies+i, circles+i, wp, f2, 0.0f, friction, restitution, 0.0f, 1000.0f, 8.0f);
    ////}

    ////for (spInt i = 20; i < 24; ++i)
    ////{
    ////    spFloat f = ((spFloat)i - 20.0f);
    ////    spFloat f2 = ((spFloat)i - 20.0f) * 25.0f - 37.5f;
    ////    create_circle(bodies+i, circles+i, wp, f2, -35.0f, friction, restitution, 0.0f, 1000.0f, 12.0f);
    ////}

    ////for (spInt i = 24; i < 32; ++i)
    ////{
    ////    spFloat f = ((spFloat)i - 24.0f);
    ////    spFloat f2 = ((spFloat)i - 24.0f) * 10.0f - 37.5f;
    ////    create_circle(bodies+i, circles+i, wp, f2, -20.0f, friction, restitution, 0.0f, 250.0f, 3.0f);
    ////}

    ////for (spInt i = 32; i < 64; ++i)
    ////{
    ////    spFloat f = ((spFloat)i - 32.0f);
    ////    spFloat f2 = ((spFloat)i - 32.0f) * 5.0f - 37.5f;
    ////    spFloat g_scale = spClamp((f/8.0f)+0.25f, 0.0f, 1.0f);
    ////    create_circle(bodies+i, circles+i, wp, f2, 40.0f, friction, restitution, g_scale, 8.0f, 1.5f);
    ////}

    //glfwSetTime(0.0);
    //double timestep = 1.0 / 60.0;
    //while(!glfwWindowShouldClose(window))
    //{
    //    double dt = glfwGetTime() / 1000.;

    //    while (dt < timestep)
    //    {
    //        dt = glfwGetTime();
    //    }
    //    glClear(GL_COLOR_BUFFER_BIT);
    //    glMatrixMode(GL_PROJECTION);
    //    glLoadIdentity();
    //    spFloat os = 50.0f;
    //    glOrtho(-os, os, -os, os, -os, os);

    //    glMatrixMode(GL_MODELVIEW);
    //    glLoadIdentity();

    //    spWorldStep(wp, 1.0f / 60.0f);

    //    glfwSwapBuffers(window);

    //    glfwPollEvents();
    //    glfwSetTime(0.0);
    //}

    //glfwDestroyWindow(window);

    //glfwTerminate();
    //return 0;
}