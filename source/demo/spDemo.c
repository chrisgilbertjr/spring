
#include <time.h>
#include "demo\spDemo.h"

spDemo* Demo;

#define MAX_DT 0.25f
#define ALPHA 1.0f

spFloat spLineScaleSmall = 1.0f;
spFloat spLineScaleBig = 2.0f;

/// convenience function
static spVector
vec(spFloat x, spFloat y) 
{ 
    return spVectorConstruct(x, y); 
}

static void init() {};
static void update(spFloat dt) { spDrawCircle(vec(0.0f, 0.0f), 0.0f, 0.2f, RED(), WHITE()); };
static void destroy() {};

void 
spDemoSetCameraTranslation(spVector translation)
{
    Demo->view[3] = translation.x;
    Demo->view[7] = translation.y;
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

static void KeyboardDefault() {}

spDemo*
spDemoNew(initFunc init, updateFunc update, destroyFunc destroy, spFrustum frustum, spViewport view)
{
    spDemo* Demo = (spDemo*)spMalloc(sizeof(spDemo));
    Demo->world = spWorldConstruct(10, spVectorConstruct(0.0f, 0.098065f * -frustum.top+frustum.bottom));
    Demo->mouse = (spMouse){spMouseJointNew(NULL, 1.5f, 0.4f, spVectorZero(), spVectorZero()), NULL, spVectorConstruct(0.0f, 0.0f) };
    Demo->window = NULL;
    Demo->initialize = init;
    Demo->update = update;
    Demo->destroy = destroy;
    Demo->keyboard = KeyboardDefault;
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
spDemoFree(spDemo* demo)
{
    NULLCHECK(demo);
    free(demo);
}

static spVector
MousePosition()
{
    double x, y;
    glfwGetCursorPos(Demo->window, &x, &y);
    return spVectorConstruct((spFloat)x, (spFloat)Demo->viewport.height - (spFloat)y);
}

static spVector
ScreenToWorld(spVector position)
{
    return spDeproject(position, Demo->view, Demo->ortho, Demo->viewport);
}

static void
Resize(spWindow* window, int width, int height)
{
    Demo->viewport.width = width;
    Demo->viewport.height = height;
    glViewport(0, 0, width, height);
}

static void
Keyboard(spWindow* window, int key, int scancode, int action, int mods)
{
    Demo->keyboard();
}

static void
MouseClick(spWindow* window, int button, int state, int mods)
{
    spMouse* mouse = &Demo->mouse;

    if (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_PRESS)
    {
        if (mouse->shape != NULL) return;

        spShape* shape = spWorldTestPoint(&Demo->world, mouse->position);
        if (shape != NULL && shape->body->type == SP_BODY_DYNAMIC)
        {
            spMouseJointStart(mouse->constraint, shape->body, mouse->position);
            spWorldAddConstraint(&Demo->world, mouse->constraint);
            mouse->shape = shape;
        }
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT && state == GLFW_RELEASE)
    {
        if (mouse->shape != NULL)
        {
            spMouseJointEnd(mouse->constraint);
            spWorldRemoveConstraint(&Demo->world, mouse->constraint);
            mouse->shape = NULL;
        }
    }
}

static void
Mouse(spWindow* window, spVector position)
{
    spMouse* mouse = &Demo->mouse;

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
    Demo->window = glfwCreateWindow(Demo->viewport.width, Demo->viewport.height, "Simple example", NULL, NULL);

    if (!Demo->window)
    {
        /// TODO: add compat profile/draw functions if core profile doesnt work
        glfwTerminate();
        spAssert(spFalse, "GLFW window creation failed.\n");
    }

    /// set GLFW callback funcs
    glfwSetMouseButtonCallback(Demo->window, (GLFWmousebuttonfun)MouseClick);
    glfwSetWindowSizeCallback(Demo->window, (GLFWwindowsizefun)Resize);
    glfwSetKeyCallback(Demo->window, (GLFWkeyfun)Keyboard);

    /// make the OpenGL context current, and set the time to 0.0
    glfwMakeContextCurrent(Demo->window);
    glfwSetTime(0.0f);
}

static void
Initialize(spDemo* demo)
{
    Demo = demo;
    SetupGLFW();
    spDrawInit();
    Demo->initialize();
}

static void
TickPhysics()
{
    Demo->timeAccum -= Demo->timestep;

    if (Demo->paused) return;

    spClearBuffers();

    Demo->update(Demo->timestep);
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
    while (dt < Demo->timestep)
    {
        dt = (spFloat)glfwGetTime();
    }

    TickPhysics();

    /// i dont use a glfw callback since it doesnt update every frame, only when the mouse is moved
    /// this can be bad if the mouse hasnt moved, but the camera is
    Mouse(Demo->window, MousePosition());

    glfwSetTime(0.0f);

    Demo->timePrev = Demo->time;
}

static void
Render()
{
    spDrawDemo();

    glfwSwapBuffers(Demo->window);
    glfwPollEvents();
}

static void
Destroy()
{
    Demo->destroy();
}

void
spDemoInitRandomSeed()
{
    srand((unsigned int)time(NULL));
}

spFloat
spDemoRandomFloatRange(float min, float max) 
{
    float random = ((float)rand()) / (float)RAND_MAX;
    float range = max - min;
    return min + random * range;
}

spColor 
spDemoRandomColor()
{
    spColor color;
    color.r = spDemoRandomFloatRange(0.f, 1.f);
    color.g = spDemoRandomFloatRange(0.f, 1.f);
    color.b = spDemoRandomFloatRange(0.f, 1.f);
    color.a = 1.0f;
    return color;
}

spColor 
spDemoRandomPastelColor()
{
    spColor pastel = { 1.0f, 1.0f, 1.0f, 1.0f };
    spColor color = spDemoRandomColor();
    color.r = (pastel.r + color.r) * 0.5f;
    color.g = (pastel.g + color.g) * 0.5f;
    color.b = (pastel.b + color.b) * 0.5f;
    return color;
}

spBool
spDemoKeyPressed(spKey key)
{
    return glfwGetKey(Demo->window, key) == GLFW_PRESS;
}

spBool 
spDemoKeyReleased(spKey key)
{
    return glfwGetKey(Demo->window, key) == GLFW_RELEASE;
}

void 
spDemoDrawSingleBody(spSingleBodyObject* object)
{
    spDemoDrawShape(object->shape, object->color, object->border);
}

void 
spDemoDrawMultiBody(spMultiBodyObject* object)
{
    for (spInt i = 0; i < object->count; ++i)
    {
        spDemoDrawShape(object->shapes[i], object->colors[i], object->borders[i]);
    }
}

void 
spDemoDrawShape(spShape* shape, spColor color, spColor border)
{
    if (spShapeIsCircle(shape))
    {
        spDemoDrawCircle(shape, color, border);
    }
    else if (spShapeIsPolygon(shape))
    {
        spDemoDrawPolygon(shape, color, border);
    }
    else if (spShapeIsSegment(shape))
    {
        spDemoDrawSegment(shape, color, border);
    }
}

void 
spDemoDrawCircle(spShape* shape, spColor color, spColor border)
{
    /// check if the shape is a circle
    spAssert(spShapeIsCircle(shape), "shape is not a circle!\n");

    /// the shape is a circle, get a circle pointer
    spCircle* circle = spShapeCastCircle(shape);

    /// get the circles transform
    spTransform* xf = &shape->body->xf;

    /// get the circles center in world space and create a lne from the center to the outside of the circle
    spVector pos = spMultXformVec(*xf, circle->center);
    spFloat scale = 0.95f;
    spVector line = spMultXformVec(*xf, spAddVecs(circle->center, spVectorConstruct(0.0f, circle->radius*scale)));
    spFloat radius = circle->radius;

    /// draw the circle
    spDrawCircle(pos, spRotationGetAngleDeg(xf->q), radius, color, border);
    spDrawSegment(line, pos, spLineScaleSmall * 0.75f, border, border);
}

void 
spDemoDrawPolygon(spShape* shape, spColor color, spColor border)
{
    /// check if the shape is a polygon
    spAssert(spShapeIsPolygon(shape), "shape is not a polygon!\n");

    /// get the poly pointer
    spPolygon* poly = spShapeCastPolygon(shape);

    /// get the world space COM
    spTransform* xf = &shape->body->xf;
    spVector center = spMultXformVec(*xf, spShapeGetCOM(shape));
    spVector vertices[32];
    spEdge* edges = poly->edges;
    spInt count = poly->count;

    /// get all the poly vertices in world space
    for (spInt i = 0; i < count; ++i)
    {
        vertices[i] = spMultXformVec(*xf, edges[i].vertex);
    }

    /// draw the polygon
    spDrawPolygon(vec(0.0f, 0.0f), 0.0f, vertices, count, center, color, border);
}

void 
spDemoDrawSegment(spShape* shape, spColor color, spColor border)
{
    /// check if the shape is a segment
    spAssert(spShapeIsSegment(shape), "shape is not a segment!\n");

    /// get a segment pointer and its transform
    spSegment* segment = spShapeCastSegment(shape);
    spTransform* xf = &shape->body->xf;

    /// get the segment endpoints and radius
    spFloat radius = segment->radius * 2.0f;
    spVector pointA = spMultXformVec(*xf, segment->pointA);
    spVector pointB = spMultXformVec(*xf, segment->pointB);

    pointA = segment->pointA;
    pointB = segment->pointB;

    /// draw the segment
    spDrawSegment(pointA, pointB, radius, color, border);
}

void 
spDemoDrawGearJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// check if the constraint is a gear joint
    spAssert(spConstraintIsGearJoint(constraint), "constraint is not a gear joint!\n");

    /// get the gear joint bodies
    spBody* bodyA = spConstraintGetBodyA(constraint);
    spBody* bodyB = spConstraintGetBodyB(constraint);

    /// get the gear joint points in world space
    spVector pointA = spBodyGetPosition(bodyA);
    spVector pointB = spBodyGetPosition(bodyB);

    /// get the color and border colors
    spColor Color = color;
    spColor Border = border;

    /// scale the opacity down of each
    Color.a = color.a * 0.5f;
    Border.a = border.a * 0.5f;

    /// draw the gear joint
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, Color, Border);
    spDrawCircle(pointB, 0.0f, spLineScaleSmall, Color, Border);
    spDrawLine(pointA, pointB, spLineScaleSmall, Color, Border);
}

void 
spDemoDrawAngularSpringJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// check if the constraint is an angular spring
    spAssert(spConstraintIsAngularSpringJoint(constraint), "constraint is not an angular spring joint!\n");

    /// get the two bodies
    spBody* bodyA = spConstraintGetBodyA(constraint);
    spBody* bodyB = spConstraintGetBodyB(constraint);

    /// get the two world points
    spVector pointA = spBodyGetPosition(bodyA);
    spVector pointB = spBodyGetPosition(bodyB);

    /// get the color and border colors
    spColor Color = color;
    spColor Border = border;

    /// scale the opacity down of each
    Color.a = color.a * 0.5f;
    Border.a = border.a * 0.5f;;

    /// draw the angular spring
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, Color, Border);
    spDrawCircle(pointB, 0.0f, spLineScaleSmall, Color, Border);
    spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleBig, Color, Border);
}

void 
spDemoDrawMotorJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// check if the constraint is a motor joint
    spAssert(spConstraintIsMotorJoint(constraint), "constraint is not a motor joint!\n");

    spBody* bodyA = spConstraintGetBodyA(constraint);
    spBody* bodyB = spConstraintGetBodyB(constraint);

    /// get the two anchor points in world space
    spVector pointA = spBodyGetPosition(bodyA);
    spVector pointB = spBodyGetPosition(bodyB);

    /// get the color and border colors
    spColor Color = color;
    spColor Border = border;

    /// scale the opacity down of each
    Color.a = color.a * 0.5f;
    Border.a = border.a * 0.5f;;

    /// draw the motor joint
    spDrawSegment(pointA, pointB, spLineScaleSmall, Color, Border);
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, Color, Border);
    spDrawCircle(pointB, 0.0f, spLineScaleSmall, Color, Border);
}

void
spDemoDrawMouseJoint(spConstraint* constraint, spColor color, spColor cursor, spColor border)
{
    /// cast the mouse joint safely
    spMouseJoint* mouseJoint = spConstraintCastMouseJoint(constraint);

    /// get the mouse joints body
    spBody*  bodyA = spConstraintGetBodyA(constraint);

    /// compute the two world anchors to draw
    spVector pointA = spMultXformVec(bodyA->xf, mouseJoint->anchor);
    spVector pointB = mouseJoint->target;

    /// draw the points, a spring, and a large circle to represent the circle
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, color, border);
    spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleSmall, color, border);
    spDrawCircle(pointB, 0.0f, spLineScaleBig, cursor, border);
}

void 
spDemoDrawRopeJoint(spConstraint* constraint, spColor circles, spColor rope, spColor border)
{
    /// check if this is really a rope joint
    spAssert(spConstraintIsRopeJoint(constraint), "constraint is not a rope joint!\n");

    /// get the anchors in world space
    spVector pointA = spRopeJointGetWorldAnchorA(constraint);
    spVector pointB = spRopeJointGetWorldAnchorB(constraint);

    /// draw the rope joint
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, circles, border);
    spDrawCircle(pointB, 0.0f, spLineScaleSmall, circles, border);
    spDrawRope(pointA, pointB, 8, spLineScaleSmall, rope, circles, border);
}

void 
spDemoDrawDistanceJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// check if this is really a distance joint
    spAssert(spConstraintIsDistanceJoint(constraint), "constraint is not a distance joint!\n");

    /// get the anchors in world space
    spVector pointA = spDistanceJointGetWorldAnchorA(constraint);
    spVector pointB = spDistanceJointGetWorldAnchorB(constraint);

    /// draw the distance constraint
    spDrawSegment(pointA, pointB, spLineScaleSmall * 1.5f, color, border);
}

void 
spDemoDrawPointJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// check if this is really a point joint
    spAssert(spConstraintIsPointJoint(constraint), "constraint is not a point joint!\n");

    /// get the anchors in world space
    spVector pointA = spPointJointGetWorldAnchorA(constraint);
    spVector pointB = spPointJointGetWorldAnchorB(constraint);

    /// get the bodies COM in world space
    spVector comA = spBodyGetPosition(constraint->bodyA);
    spVector comB = spBodyGetPosition(constraint->bodyB);

    /// draw the point joint
    spDrawCircle(comA, 0.0f, spLineScaleSmall, color, border);
    spDrawCircle(comB, 0.0f, spLineScaleSmall, color, border);
    spDrawLine(pointA, comA, spLineScaleSmall, color, border);
    spDrawLine(pointB, comB, spLineScaleSmall, color, border);
    spDrawCircle(pointB, 0.0f, spLineScaleBig, color, color);
}

void 
spDemoDrawSpringJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// check if this is really a spring joint
    spAssert(spConstraintIsSpringJoint(constraint), "constraint is not a spring joint!\n");

    /// get the anchors in world space
    spVector pointA = spSpringJointGetWorldAnchorA(constraint);
    spVector pointB = spSpringJointGetWorldAnchorB(constraint);

    /// draw the spring joint
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, color, border);
    spDrawCircle(pointB, 0.0f, spLineScaleSmall, color, border);
    spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleBig, color, border);
}

void 
spDemoDrawWheelJoint(spConstraint* constraint, spColor color, spColor border)
{
    /// cast the wheel joint safely
    spWheelJoint* wheelJoint = spConstraintCastWheelJoint(constraint);

    /// get the constraints bodies
    spBody* bodyA = spConstraintGetBodyA(constraint);
    spBody* bodyB = spConstraintGetBodyB(constraint);

    /// reflect the springs anchor points around the wheel to look more plausible
    spVector normal = wheelJoint->tWorld;
    spVector direction = spMultRotVec(bodyA->xf.q, wheelJoint->anchorA);
    spFloat  dist = spDot(normal, direction);

    /// get the two spring anchors in world space
    spVector pointB = spMultXformVec(bodyB->xf, wheelJoint->anchorB);
    spVector pointA = spAddVecs(spMultVecFlt(normal, dist), bodyA->p);

    /// draw the wheel joint
    spDrawCircle(pointA, 0.0f, spLineScaleSmall, color, border);
    spDrawCircle(pointB, 0.0f, spLineScaleSmall, color, border);
    spDrawSpring(pointA, pointB, spLineScaleSmall, spLineScaleBig, color, border);

    color.a = color.a * 0.5f;
    spDrawSegment(pointA, pointB, spLineScaleSmall, color, color);
}

void 
spDemoDrawConstraint(spConstraint* constraint)
{
    if (spConstraintIsGearJoint(constraint))
    {
        spDemoDrawGearJoint(constraint, RGBA(1,0,1,.5), RGBA(0,0,0,.5));
        //spColor color = RGBA(1.0,0.0,1.0,0.5f);
        //spColor black = RGBA(0.0,0.0,0.0,0.5f);
    }
    else if (spConstraintIsAngularSpringJoint(constraint))
    {
        spDemoDrawAngularSpringJoint(constraint, RGBA(1,0,1,.5f), RGBA(0,0,0,0.5));
        //spColor color = RGBA(1.0,0.0,1.0,0.5f);
        //spColor black = RGBA(0.0,0.0,0.0,0.5f);
    }
    else if (spConstraintIsMotorJoint(constraint))
    {
        spDemoDrawMotorJoint(constraint, RGBA(1,.5,0,.5f), RGBA(0,0,0,.5));
        //spColor color = RGBA(1.0,0.5,0.0,0.5f);
        //spColor black = RGBA(0.0,0.0,0.0,0.5f);
    }
    else if (spConstraintIsMouseJoint(constraint))
    {
        spDemoDrawMouseJoint(constraint, RGB(0,1,0), WHITE(), BLACK());
    }
    else if (spConstraintIsRopeJoint(constraint))
    {
        spDemoDrawRopeJoint(constraint, RGB(1,1,0), RGB(.5,.5,0), BLACK());
    }
    else if (spConstraintIsDistanceJoint(constraint))
    {
        spDemoDrawDistanceJoint(constraint, RGB(.8,0,0), BLACK());
    }
    else if (spConstraintIsPointJoint(constraint))
    {
        spDemoDrawPointJoint(constraint, RGB(0,.5,1), BLACK());
    }
    else if (spConstraintIsSpringJoint(constraint))
    {
        spDemoDrawSpringJoint(constraint, RGB(0,.8,0), BLACK());
    }
    else if (spConstraintIsWheelJoint(constraint))
    {
        spDemoDrawWheelJoint(constraint, RGB(0,1,0), BLACK());
    }
}

void spDemoRun(spDemo* demo)
{
    Initialize(demo);

    while(!glfwWindowShouldClose(demo->window))
    {
        Update();
        Render();
    }

    Destroy();

    spDemoFree(demo);
}