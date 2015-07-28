
#include "spDemo.h"

#include "spPolygon.h"
#include "spSegment.h"
#include "spCircle.h"

spDemo* demo;

#define MAX_DT 0.25f
#define ALPHA 1.0f

spFloat spLineScaleSmall = 1.0f;
spFloat spLineScaleBig = 2.0f;

static void init() {};
static void update(spFloat dt) { spDrawCircle({0.0f, 0.0f}, 0.0f, 0.2f, RED(), WHITE()); };
static void destroy() {};

void 
spDemoSetCameraTranslation(spVector translation)
{
    demo->view[3] = translation.x;
    demo->view[7] = translation.y;
}

static void 
DemoOrthoMatrix(spFloat* ortho, spFrustum* frustum)
{
    ortho[0]  = 2.0f/(frustum->right - frustum->left);
    ortho[1]  = 0.0f;
    ortho[2]  = 0.0f;
    ortho[3]  = -(frustum->right + frustum->left)/(frustum->right - frustum->left);
    ortho[4]  = 0.0f;
    ortho[5]  = 2.0f/(frustum->top - frustum->bottom);
    ortho[6]  = 0.0f;
    ortho[7]  = -(frustum->top + frustum->bottom)/(frustum->top - frustum->bottom);
    ortho[8]  = 0.0f;              
    ortho[9]  = 0.0f;           
    ortho[10] = 2.0f/(frustum->far - frustum->near);
    ortho[11] = -(frustum->far + frustum->near)/(frustum->far - frustum->near);
    ortho[12] = 0.0f;              
    ortho[13] = 0.0f;           
    ortho[14] = 0.0f;             
    ortho[15] = 1.0f;
}

static void
DemoViewMatrix(spFloat* view, spVector translation)
{
    view[0]  = 1.0f;
    view[1]  = 0.0f; 
    view[2]  = 0.0f; 
    view[3]  = translation.x;
    view[4]  = 0.0f; 
    view[5]  = 1.0f; 
    view[6]  = 0.0f; 
    view[7]  = translation.y;
    view[8]  = 0.0f; 
    view[9]  = 0.0f; 
    view[10] = 1.0f; 
    view[11] = 0.0f; 
    view[12] = 0.0f;
    view[13] = 0.0f;
    view[14] = 0.0f; 
    view[15] = 1.0f; 
}

spDemo*
spDemoNew(initFunc init, updateFunc update, destroyFunc destroy, spFrustum frustum, spViewport view)
{
    spDemo* Demo = (spDemo*)spMalloc(sizeof(spDemo));
    Demo->world = spWorldConstruct(10, spVectorConstruct(0.0f, 0.098065f * -frustum.top+frustum.bottom));
    Demo->mouse = { spMouseJointNew(NULL, 1.5f, 0.4f, spVectorZero(), spVectorZero()), NULL, spVectorConstruct(0.0f, 0.0f) };
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
    Demo->frustum = frustum;
    Demo->viewport = view;
    DemoOrthoMatrix(Demo->ortho, &frustum);
    DemoViewMatrix(Demo->view, spVectorZero());
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
    glfwGetCursorPos(demo->window, &x, &y);
    return spVectorConstruct((spFloat)x, (spFloat)demo->viewport.height - (spFloat)y);
}

static spVector
ScreenToWorld(spVector position)
{
    return spDeproject(position, demo->view, demo->ortho, demo->viewport);
}

static void
Resize(spWindow* window, int width, int height)
{
    demo->viewport.width = width;
    demo->viewport.height = height;
    glViewport(0, 0, width, height);
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
Mouse(spWindow* window, spVector position)
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
    demo->window = glfwCreateWindow(demo->viewport.width, demo->viewport.height, "Simple example", NULL, NULL);

    if (!demo->window)
    {
        /// TODO: add compat profile/draw functions if core profile doesnt work
        glfwTerminate();
        spAssert(spFalse, "GLFW window creation failed.\n");
    }

    glfwSetMouseButtonCallback(demo->window, (GLFWmousebuttonfun)MouseClick);
    glfwSetWindowSizeCallback(demo->window, (GLFWwindowsizefun)Resize);

    glfwMakeContextCurrent(demo->window);
    glfwSetTime(0.0f);
}

static void
Initialize()
{
    demo = vehicle;
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

    /// i dont use a glfw callback since it doesnt update every frame, only when the mouse is moved
    /// this can be bad if the mouse hasnt moved, but the camera is
    Mouse(demo->window, MousePosition());

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
    spColor Color = color;
    spColor Border = border;
    Color.a = Border.a = ALPHA;

    if (shape->type == SP_SHAPE_CIRCLE)
    {
        spCircle* circle = spShapeCastCircle(shape);
        spTransform* xf = &shape->body->xf;
        spVector pos = spMult(*xf, circle->center);
        spFloat scale = 0.95f;
        spVector line = spMult(*xf, spAdd(circle->center, spVectorConstruct(0.0f, circle->radius*scale)));
        spFloat radius = circle->radius;
        spDrawCircle(pos, spRotationGetAngleDeg(xf->q), radius, Color, Border);
        spDrawSegment(line, pos, spLineScaleSmall, Border, Border);
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

        spDrawPolygon({0.0f, 0.0f}, 0.0f, vertices, 4, center, Color, Border);
    }
    if (shape->type == SP_SHAPE_SEGMENT)
    {
        spSegment* segment = spShapeCastSegment(shape);
        spTransform* xf = &shape->body->xf;

        spFloat radius = segment->radius * 2.0f;
        spVector pointA = spMult(*xf, segment->pointA);
        spVector pointB = spMult(*xf, segment->pointB);

        pointA = segment->pointA;
        pointB = segment->pointB;

        spDrawSegment(pointA, pointB, radius, Color, Border);
    }
}

void 
spDemoDrawConstraint(spConstraint* constraint)
{
    if (spConstraintIsMouseJoint(constraint))
    {
        spMouseJoint* mouseJoint = spConstraintCastMouseJoint(constraint);
        spBody* bodyA = mouseJoint->constraint.bodyA;
        spVector pointA = spMult(bodyA->xf, mouseJoint->anchor);
        spVector pointB = mouseJoint->target;
        spDrawCircle(pointA, 0.0f, spLineScaleSmall, {0.0f, 1.0f, 0.0f, 1.0f}, BLACK());
        spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleSmall, {0.0f, 1.0f, 0.0f, 1.0f}, BLACK());
        spDrawCircle(pointB, 0.0f, spLineScaleBig, WHITE(), BLACK());
    }
    else if(spConstraintIsRopeJoint(constraint))
    {
        spRopeJoint* ropeJoint = spConstraintCastRopeJoint(constraint);

        spBody* bodyA = ropeJoint->constraint.bodyA;
        spBody* bodyB = ropeJoint->constraint.bodyB;

        spVector pointA = spMult(bodyA->xf, ropeJoint->anchorA);
        spVector pointB = spMult(bodyB->xf, ropeJoint->anchorB);

        spDrawCircle(pointA, 0.0f, spLineScaleSmall, RGB(1,1,0), BLACK());
        spDrawCircle(pointB, 0.0f, spLineScaleSmall, RGB(1,1,0), BLACK());
        spDrawRope(pointA, pointB, 8, spLineScaleSmall, RGB(0.5f,0.5f,0), RGB(1.0f,1.0f,0), BLACK());
    }
    else if (spConstraintIsDistanceJoint(constraint))
    {
        spDistanceJoint* distanceJoint = spConstraintCastDistanceJoint(constraint);

        spBody* bodyA = distanceJoint->constraint.bodyA;
        spBody* bodyB = distanceJoint->constraint.bodyB;

        spVector pointA = spMult(bodyA->xf, distanceJoint->anchorA);
        spVector pointB = spMult(bodyB->xf, distanceJoint->anchorB);

        spDrawSegment(pointA, pointB, spLineScaleSmall * 1.5f, RGB(0.8,0,0), BLACK());
    }
    else if (spConstraintIsPointJoint(constraint))
    {
        spPointJoint* pointJoint = spConstraintCastPointJoint(constraint);

        spBody* bodyA = pointJoint->constraint.bodyA;
        spBody* bodyB = pointJoint->constraint.bodyB;

        spVector pointA = spMult(bodyA->xf, pointJoint->anchorA);
        spVector pointB = spMult(bodyB->xf, pointJoint->anchorB);

        spDrawCircle(bodyB->p, 0.0f, spLineScaleSmall, RGB(0,0.5f,1), BLACK());
        spDrawCircle(bodyA->p, 0.0f, spLineScaleSmall, RGB(0,0.5f,1), BLACK());
        spDrawLine(pointA, bodyA->p, spLineScaleSmall, RGBA(0.0f, 0.5f, 1.0f, 1.0f), BLACK());
        spDrawLine(pointB, bodyB->p, spLineScaleSmall, RGBA(0.0f, 0.5f, 1.0f, 1.0f), BLACK());
        spDrawCircle(pointB, 0.0f, spLineScaleBig, RGB(0,0.5f,1), RGBA(0.0f, 0.5, 1.0, 1.0));
    }
    else if (spConstraintIsSpringJoint(constraint))
    {
        spSpringJoint* springJoint = spConstraintCastSpringJoint(constraint);

        spBody* bodyA = springJoint->constraint.bodyA;
        spBody* bodyB = springJoint->constraint.bodyB;

        spVector pointA = spMult(bodyA->xf, springJoint->anchorA);
        spVector pointB = spMult(bodyB->xf, springJoint->anchorB);

        spDrawCircle(pointA, 0.0f, spLineScaleSmall, RGB(0,0.8,0), BLACK());
        spDrawCircle(pointB, 0.0f, spLineScaleSmall, RGB(0,0.8,0), BLACK());
        spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleBig, RGB(0,0.8,0), BLACK());
    }
    else if (spConstraintIsWheelJoint(constraint))
    {
        spWheelJoint* wheelJoint = spConstraintCastWheelJoint(constraint);

        spBody* bodyA = wheelJoint->constraint.bodyA;
        spBody* bodyB = wheelJoint->constraint.bodyB;

        spVector normal = wheelJoint->tWorld;
        spVector direction = spMult(bodyA->xf.q, wheelJoint->anchorA);
        spFloat  dist = spDot(normal, direction);

        spVector pointB = spMult(bodyB->xf, wheelJoint->anchorB);
        spVector pointA = spAdd(spMult(normal, dist), bodyA->p);

        spColor c = RGB(0,1,0);
        spDrawCircle(pointA, 0.0f, spLineScaleSmall, c, BLACK());
        spDrawCircle(pointB, 0.0f, spLineScaleSmall, c, BLACK());
        spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleBig, c, BLACK());

        spColor color = RGBA(0.0,1.0,0.0,0.5f);

        spDrawSegment(pointA, pointB, spLineScaleSmall, color, color);
    }
    else if (spConstraintIsGearJoint(constraint))
    {
        spGearJoint* gearJoint = spConstraintCastGearJoint(constraint);

        spBody* bodyA = gearJoint->constraint.bodyA;
        spBody* bodyB = gearJoint->constraint.bodyB;

        spVector pointA = bodyA->xf.p;
        spVector pointB = bodyB->xf.p;

        spColor color = RGBA(1.0,0.0,1.0,0.5f);
        spColor black = RGBA(0.0,0.0,0.0,0.5f);

        spDrawCircle(pointA, 0.0f, spLineScaleSmall, color, black);
        spDrawCircle(pointB, 0.0f, spLineScaleSmall, color, black);
        spDrawLine(pointA, pointB, spLineScaleSmall, color, black);
    }
    else if (spConstraintIsAngularSpringJoint(constraint))
    {
        spAngularSpringJoint* angSpringJoint = spConstraintCastAngularSpringJoint(constraint);

        spBody* bodyA = angSpringJoint->constraint.bodyA;
        spBody* bodyB = angSpringJoint->constraint.bodyB;

        spVector pointA = bodyA->xf.p;
        spVector pointB = bodyB->xf.p;

        spColor color = RGBA(1.0,0.0,1.0,0.5f);
        spColor black = RGBA(0.0,0.0,0.0,0.5f);

        spDrawCircle(pointA, 0.0f, spLineScaleSmall, color, black);
        spDrawCircle(pointB, 0.0f, spLineScaleSmall, color, black);
        spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleBig, color, black);
    }
    else if (spConstraintIsMotorJoint(constraint))
    {
        spMotorJoint* motorJoint = spConstraintCastMotorJoint(constraint);

        spBody* bodyA = motorJoint->constraint.bodyA;
        spBody* bodyB = motorJoint->constraint.bodyB;

        spVector pointA = bodyA->xf.p;
        spVector pointB = bodyB->xf.p;

        spColor color = RGBA(1.0,0.5,0.0,0.5f);
        spColor black = RGBA(0.0,0.0,0.0,0.5f);

        spDrawSegment(pointA, pointB, spLineScaleSmall, color, black);
        spDrawCircle(pointA, 0.0f, spLineScaleSmall, color, black);
        spDrawCircle(pointB, 0.0f, spLineScaleSmall, color, black);
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