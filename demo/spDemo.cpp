
#include "spDemo.h"

spDemo* demo;

#define MAX_DT 0.25f

static void init() {};
static void update(spFloat dt) { spDrawCircle({0.0f, 0.0f}, 0.0f, 0.2f, RED(), WHITE()); };
static void destroy() {};

spDemo*
spDemoNew(initFunc init, updateFunc update, destroyFunc destroy)
{
    spDemo* Demo = (spDemo*)spMalloc(sizeof(spDemo));
    Demo->world = spWorldConstruct(10, spVectorConstruct(0.0f, 0.0f));
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
    demo->time = (spFloat)glfwGetTime();
    spFloat dt = demo->time - demo->timePrev;

    demo->timeAccum += dt > MAX_DT ? MAX_DT : dt;
    //while(demo->timeAccum > demo->timestep)
    //{
        TickPhysics();
    //}

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
spDemoDrawPolygon(spPolygon* poly, spTransform* xf)
{
}

void 
spDemoDrawCircle(spCircle* circle, spTransform* xf)
{
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