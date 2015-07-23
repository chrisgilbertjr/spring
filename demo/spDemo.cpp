
#include "spDemo.h"

#include "spPolygon.h"
#include "spSegment.h"
#include "spCircle.h"

spDemo* demo;

#define MAX_DT 0.25f

static void init() {};
static void update(spFloat dt) { spDrawCircle({0.0f, 0.0f}, 0.0f, 0.2f, RED(), WHITE()); };
static void destroy() {};

spDemo*
spDemoNew(initFunc init, updateFunc update, destroyFunc destroy)
{
    spDemo* Demo = (spDemo*)spMalloc(sizeof(spDemo));
    Demo->world = spWorldConstruct(10, spVectorConstruct(0.0f, -98.0f));
    Demo->mouse = { spMouseJointNew(NULL, 1.5f, 0.5f, spVectorZero(), spVectorZero()), NULL, spVectorConstruct(0.0f, 0.0f) };
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

static spVector
MousePosition()
{
    double x, y;
    int xx, yy;
    glfwGetCursorPos(demo->window, &x, &y);
    glfwGetWindowSize(demo->window, &xx, &yy);
    return spVectorConstruct((spFloat)x, (spFloat)yy - (spFloat)y);
}

static spVector
ScreenToWorld(spVector position)
{
    spFloat IDENTITY[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f,
                            0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f};
    spFloat size = 100.0f;
    spFloat right =  size;
    spFloat left  = -size;
    spFloat top   =  size;
    spFloat bot   = -size;
    spFloat near  = -size;
    spFloat far   =  size;

    spFloat ortho[16] = {
        2.0f/(right-left), 0.0f,           0.0f,            -(right+left)/(right-left),
        0.0f,              2.0f/(top-bot), 0.0f,            -(top+bot)/(top-bot),
        0.0f,              0.0f,           2.0f/(far-near), -(far+near)/(far-near),
        0.0f,              0.0f,           0.0f,             1.0f };

    int ww, wh;
    glfwGetWindowSize(demo->window, &ww, &wh);

    spViewport view = {800.0f, 800.0f};

    return spDeproject(position, IDENTITY, ortho, view);
}

static void
MouseClick(spWindow* window, int button, int state, int mods)
{
    spMouse* mouse = &demo->mouse;

    if (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_PRESS)
    {
        if (mouse->shape != NULL) return;

        spShape* shape = spWorldTestPoint(&demo->world, mouse->position);
        if (shape != NULL && shape->body->type == SP_BODY_DYNAMIC)
        {
            spMouseJointStart(mouse->constraint, shape->body, mouse->position);
            spWorldAddConstraint(&demo->world, mouse->constraint);
            mouse->shape = shape;
        }
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_RELEASE)
    {
        if (mouse->shape != NULL)
        {
            spMouseJointEnd(mouse->constraint);
            spWorldRemoveConstraint(&demo->world, mouse->constraint);
            mouse->shape = NULL;
        }
    }
}

static void
Mouse(spWindow* window, double, double)
{
    spMouse* mouse = &demo->mouse;

    mouse->position = ScreenToWorld(MousePosition());

    if (mouse->shape != NULL)
    {
        spMouseJointSetTarget(mouse->constraint, mouse->position);
    }
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
    
    /// create the window
    demo->window = glfwCreateWindow(800, 800, "Simple example", NULL, NULL);

    if (!demo->window)
    {
        /// TODO: add compat profile/draw functions if core profile doesnt work
        glfwTerminate();
        spAssert(spFalse, "GLFW window creation failed.\n");
    }

    glfwSetMouseButtonCallback(demo->window, (GLFWmousebuttonfun)MouseClick);
    glfwSetCursorPosCallback(demo->window, (GLFWcursorposfun)Mouse);

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
spDemoDrawShape(spShape* shape, spColor color, spColor border)
{
    if (shape->type == SP_SHAPE_CIRCLE)
    {
        spCircle* circle = spShapeCastCircle(shape);
        spTransform* xf = &shape->body->xf;
        spVector pos = spMult(*xf, circle->center);
        spFloat scale = 0.95f;
        spVector line = spMult(*xf, spAdd(circle->center, spVectorConstruct(0.0f, circle->radius*scale)));
        spFloat radius = circle->radius;
        spDrawCircle(pos, spRotationGetAngleDeg(xf->q), radius, color, border);
        spDrawLine(line, pos, 1.0f, border, border);
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

        spDrawPolygon({0.0f, 0.0f}, 0.0f, vertices, 4, center, color, border);
    }
}

void 
spDemoDrawConstraint(spConstraint* constraint, spColor color, spColor border)
{
    if (spConstraintIsMouseJoint(constraint))
    {
        spMouseJoint* mouseJoint = spConstraintCastMouseJoint(constraint);
        spBody* bodyA = mouseJoint->constraint.bodyA;
        spVector pointA = spMult(bodyA->xf, mouseJoint->anchor);
        spVector pointB = mouseJoint->target;
        spDrawCircle(pointA, 0.0f, 1.5f, {0.0f, 1.0f, 0.0f, 1.0f}, BLACK());
        spDrawSpring(pointA, pointB, 1.5f, 2.0f, {0.0f, 1.0f, 0.0f, 1.0f}, BLACK());
        spDrawCircle(pointB, 0.0f, 2.0f, WHITE(), BLACK());
    }
    else if(spConstraintIsRopeJoint(constraint))
    {
        spRopeJoint* ropeJoint = spConstraintCastRopeJoint(constraint);

        spBody* bodyA = ropeJoint->constraint.bodyA;
        spBody* bodyB = ropeJoint->constraint.bodyB;

        spVector pointA = spMult(bodyA->xf, ropeJoint->anchorA);
        spVector pointB = spMult(bodyB->xf, ropeJoint->anchorB);

        spDrawCircle(pointA, 0.0f, 1.5f, RGB(1,1,0), BLACK());
        spDrawCircle(pointB, 0.0f, 1.5f, RGB(1,1,0), BLACK());
        spDrawRope(pointA, pointB, 8, 1.5f, RGB(0.5f,0.5f,0), RGB(1.0f,1.0f,0), BLACK());
    }
    else if (spConstraintIsDistanceJoint(constraint))
    {
        spDistanceJoint* distanceJoint = spConstraintCastDistanceJoint(constraint);

        spBody* bodyA = distanceJoint->constraint.bodyA;
        spBody* bodyB = distanceJoint->constraint.bodyB;

        spVector pointA = spMult(bodyA->xf, distanceJoint->anchorA);
        spVector pointB = spMult(bodyB->xf, distanceJoint->anchorB);

        spDrawCircle(pointA, 0.0f, 1.5f, RGB(1,0,0), BLACK());
        spDrawCircle(pointB, 0.0f, 1.5f, RGB(1,0,0), BLACK());
        spDrawLine(pointA, pointB, 2.0f, RGB(1,0,0), BLACK());
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