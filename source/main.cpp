
#include <spring.h> 
#include "spDebugDraw.h"

#include <stdio.h>
#include <GLFW\glfw3.h>
#include "spUnitTest.h"

int main()
{
    //SP_RUN_TESTS(bound_tests);
    //SP_RUN_TESTS(chain_tests);
    //SP_RUN_TESTS(circle_tests);
    //SP_RUN_TESTS(math_tests);
    //SP_RUN_TESTS(polygon_tests);
    //SP_RUN_TESTS(shape_tests);

    GLFWwindow* window;

    if (!glfwInit())
    {
        return -1;
    }

    window = glfwCreateWindow(800, 800, "Test Application", 0, 0);
    
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    spWorld world = spWorld(spVector(0.0f, -20.0f));
    spWorld* wp = &world;
    wp->iterations = 20;

    spBody* bodya = spCreateBody(wp, SP_BODY_DYNAMIC);
    spBody* bodyb = spCreateBody(wp, SP_BODY_DYNAMIC);
    spBody* bodyc = spCreateBody(wp, SP_BODY_DYNAMIC);
    spBody* bodyd = spCreateBody(wp, SP_BODY_DYNAMIC);
    spBody* bodye = spCreateBody(wp, SP_BODY_DYNAMIC);
    spCircleDef cda;
    spCircleDef cdb;
    spCircleDef cdc;

    spFloat u = 0.01f;
    spFloat e = 0.3f;

    cda.center = spVectorZero();
    cda.mass = 100000.0f;
    cda.material.friction = u;
    cda.material.restitution = e;
    cda.radius = 7.0f;
    spBodySetPosition(bodya, spVector(-8.0f, 8.0f));
    spBodySetRotation(bodya, 45.0f);
    bodya->g_scale = 0.0f;
    spCircle* circlea = spCreateCircle(bodya, cda);

    cdb.center = spVectorZero();
    cdb.mass = 1.0f;
    cdb.material.friction = u;
    cdb.material.restitution = e;
    cdb.radius = 3.0f;
    spBodySetPosition(bodyb, spVector(0.0f, 25.0f));
    bodyb->g_scale = 5.0f;
    spCircle* circleb = spCreateCircle(bodyb, cdb);

    cdc.center = spVectorZero();
    cdc.mass = 10000.0f;
    cdc.material.friction = u;
    cdc.material.restitution = e;
    cdc.radius = 8.0f;
    spCircle* circlec = spCreateCircle(bodyc, cdc);
    cdc.radius = 3.0f;
    spCircle* circlee = spCreateCircle(bodye, cdc);
    spBodySetPosition(bodyc, spVector(7.0f, -5.0f));
    spBodySetPosition(bodye, spVector(18.0f, 4.0f));
    bodye->g_scale = 0.0f;
    bodyc->g_scale = 0.0f;

    spFloat s = 3.0f;
    spVector verts[4];
    verts[0].x = -s;  verts[0].y = -s;
    verts[1].x =  s;  verts[1].y = -s;
    verts[2].x =  s;  verts[2].y =  s;
    verts[3].x = -s;  verts[3].y =  s;

    spPolygonDef pdd;
    pdd.mass = 0.1f;
    pdd.material.friction = u;
    pdd.material.restitution = e;
    pdd.vertex_count = 4;
    pdd.vertices = verts;

    spPolygon* polygon0 = spCreatePolygon(bodyd, pdd);

    spFloat px = -1.0f;
    spFloat py = 40.0f;
    spBodySetPosition(bodyd, spVector(px, py));
    bodyd->g_scale = 1.0f;

    glfwSetTime(0.0);
    double timestep = 1.0 / 60.0;
    while(!glfwWindowShouldClose(window))
    {
        double dt = glfwGetTime() / 1000.;

        while (dt < timestep)
        {
            dt = glfwGetTime();
        }
        glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        spFloat os = 50.0f;
        glOrtho(-os, os, -os, os, -os, os);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        bool t = false;

        if (t == true)
        {

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) py += 0.1f;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) py -= 0.1f;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) px -= 0.1f;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) px += 0.1f;
        spBodySetPosition(bodyd, spVector(px, py));
        
        {
            spContact contact;
            spContactInit(&contact, spContactKey((spShape*)circlea, (spShape*)polygon0));
            spContact* c = &contact;

            spContactKey* key = &c->key; ///< contact key of the contact
            spShape*      sa  = key->shape_a;  ///< shape a of the contact
            spShape*      sb  = key->shape_b;  ///< shape b of the contact
            spBody*       ba  = sa->body;      ///< body a of the contact
            spBody*       bb  = sb->body;      ///< body b of the contact
            spTransform*  xfa = &ba->xf;       ///< transform of body a
            spTransform*  xfb = &bb->xf;       ///< transform of body b

            const spCollisionInput data = spCollisionInput(sa, sb, xfa, xfb);
            if (spCollideCirclePolygon(c, data) == spTrue)
            {
                spTransform xf = spTransform(spVectorZero(), spRotationZero());
                spDebugDrawContact(0, c, xf);
            }
        }

        {
            spContact contact;
            spContactInit(&contact, spContactKey((spShape*)circleb, (spShape*)polygon0));
            spContact* c = &contact;

            spContactKey* key = &c->key; ///< contact key of the contact
            spShape*      sa  = key->shape_a;  ///< shape a of the contact
            spShape*      sb  = key->shape_b;  ///< shape b of the contact
            spBody*       ba  = sa->body;      ///< body a of the contact
            spBody*       bb  = sb->body;      ///< body b of the contact
            spTransform*  xfa = &ba->xf;       ///< transform of body a
            spTransform*  xfb = &bb->xf;       ///< transform of body b

            const spCollisionInput data = spCollisionInput(sa, sb, xfa, xfb);
            if (spCollideCirclePolygon(c, data) == spTrue)
            {
                spTransform xf = spTransform(spVectorZero(), spRotationZero());
                spDebugDrawContact(0, c, xf);
            }
        }

        {
            spContact contact;
            spContactInit(&contact, spContactKey((spShape*)circlec, (spShape*)polygon0));
            spContact* c = &contact;

            spContactKey* key = &c->key; ///< contact key of the contact
            spShape*      sa  = key->shape_a;  ///< shape a of the contact
            spShape*      sb  = key->shape_b;  ///< shape b of the contact
            spBody*       ba  = sa->body;      ///< body a of the contact
            spBody*       bb  = sb->body;      ///< body b of the contact
            spTransform*  xfa = &ba->xf;       ///< transform of body a
            spTransform*  xfb = &bb->xf;       ///< transform of body b

            const spCollisionInput data = spCollisionInput(sa, sb, xfa, xfb);
            if (spCollideCirclePolygon(c, data) == spTrue)
            {
                spTransform xf = spTransform(spVectorZero(), spRotationZero());
                spDebugDrawContact(0, c, xf);
            }
        }

        spDebugDrawCircle(0, circlea, circlea->base_class.body->xf);
        spDebugDrawCircle(0, circleb, circleb->base_class.body->xf);
        spDebugDrawCircle(0, circlec, circlec->base_class.body->xf);
        spDebugDrawPolygon(0, polygon0, polygon0->base_class.body->xf);
        }
        else
        {
            spWorldStep(wp, 1.0f / 60.0f);
        }

        glfwSwapBuffers(window);

        glfwPollEvents();
        glfwSetTime(0.0);
    }

    glfwDestroyWindow(window);

    glfwTerminate();
    return 0;
}