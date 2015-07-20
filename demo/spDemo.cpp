
#include "spDemo.h"

#include "spPolygon.h"
#include "spSegment.h"
#include "spCircle.h"

spDemo* demo;

#define MAX_DT 0.25f

static void init() {};
static void update(spFloat dt) { spDrawCircle({0.0f, 0.0f}, 0.0f, 0.2f, RED(), WHITE()); };
static void destroy() {};

struct

spDemo*
spDemoNew(initFunc init, updateFunc update, destroyFunc destroy)
{
    spDemo* Demo = (spDemo*)spMalloc(sizeof(spDemo));
    Demo->world = spWorldConstruct(10, spVectorConstruct(0.0f, -98.0f));
    Demo->mouse = { NULL, NULL, 0.0f, 0.0f };
    Demo->window = NULL;
    Demo->initialize = init;
    Demo->update = update;
    Demo->destroy = destroy;
    Demo->background = BLACK();
    Demo->timestep = 1.0f / 60.0f;
    Demo->time= 0.0f;
    Demo->timePrev = 0.0f;
    Demo->timeAccum = 0.0f;
    Demo->paused = spFalse;
    return Demo;
}

void 
spDemoFree(spDemo** demo)
{
    NULLCHECK(*demo);
    spFree(demo);
}

static void
SetupGLFW()
{
    /// init glfw
    spAssert(glfwInit(), "Error: cannot init GLFW3\n");

    /// make sure we can get a core profile context
    //glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    demo->window = glfwCreateWindow(800, 800, "Simple example", NULL, NULL);

    if (!demo->window)
    {
        /// TODO: add compat profile/draw functions if core profile doesnt work
        glfwTerminate();
        spAssert(spFalse, "GLFW window creation failed.\n");
    }

    glfwMakeContextCurrent(demo->window);
    glfwSetTime(0.0f);
}

static void
Initialize()
{
    demo = test;
    SetupGLFW();
    spDrawInit();
    demo->initialize();
}

static void
TickPhysics()
{
    demo->timeAccum -= demo->timestep;

    if (demo->paused) return;

    spClearBuffers();

    demo->update(demo->timestep);
}

static void
Update()
{
    //demo->time = (spFloat)glfwGetTime();
    //spFloat dt = demo->time - demo->timePrev;

    //demo->timeAccum += dt > MAX_DT ? MAX_DT : dt;
    ////while(demo->timeAccum > demo->timestep)
    ////{
    ////}
    spFloat dt = (spFloat)glfwGetTime() / 1000.f;
    while (dt < demo->timestep)
    {
        dt = (spFloat)glfwGetTime();
    }
    TickPhysics();
    glfwSetTime(0.0f);

    demo->timePrev = demo->time;
}

static void
Render()
{
    spDrawDemo();

    glfwSwapBuffers(demo->window);
    glfwPollEvents();
}

static void
Destroy()
{
    demo->destroy();
}

void 
spDemoDrawShape(spShape* shape)
{
    if (shape->type == SP_SHAPE_CIRCLE)
    {
        spCircle* circle = spShapeCastCircle(shape);
        spTransform* xf = &shape->body->xf;
        spVector pos = spMult(*xf, circle->center);
        spFloat scale = 0.95f;
        spVector line0 = spMult(*xf, spAdd(circle->center, spVectorConstruct(0.0f, circle->radius*scale)));
        spVector line1 = spMult(*xf, spSub(circle->center, spVectorConstruct(0.0f, circle->radius*scale)));
        spVector line2 = spMult(*xf, spAdd(circle->center, spVectorConstruct(circle->radius*scale, 0.0f)));
        spVector line3 = spMult(*xf, spSub(circle->center, spVectorConstruct(circle->radius*scale, 0.0f)));
        spFloat radius = circle->radius;
        spDrawCircle(pos, spRotationGetAngleDeg(xf->q), radius, RED(), WHITE());
        //spDrawSegment(pos, line1, 1.0f);
        spDrawLine(line0, line1, 1.5f);
        spDrawLine(line2, line3, 1.5f);
    }
    if (shape->type == SP_SHAPE_POLYGON)
    {
        spPolygon* poly = spShapeCastPolygon(shape);
        spTransform* xf = &shape->body->xf;
        spVector center = spMult(*xf, spShapeGetCOM(shape));
        spVector vertices[4];
        spEdge* edges = poly->edges;
        spMatrix scale = spMatrixConstruct(1.0f, 0.0f, 0.0f, 1.0f);
        for (spInt i = 0; i < 4; ++i)
        {
            vertices[i] = spMult(*xf, edges[i].vertex);
            vertices[i] = spMult(scale, vertices[i]);
        }

        spDrawPolygon({0.0f, 0.0f}, 0.0f, vertices, 4, center, GREEN(), WHITE());
    }
}

void spRunDemo(spDemoIndex index)
{
    Initialize();

    while(!glfwWindowShouldClose(demo->window))
    {
        Update();
        Render();
    }

    Destroy();
}