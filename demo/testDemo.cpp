
#include "spDemo.h"
#include "spCircle.h"
#include "spPolygon.h"

struct DemoShape { spShape* shape; spBody* body; spColor color, border; };

spFilter filter = spFilterCollideAll;
DemoShape shapes[100] = {0};
spInt count = 0;

static spInt
CreateBox(spVector pos, spFloat angle, spVector size, spFloat mass, spColor color, spColor border)
{
    spVector vertices[4];
    vertices[0] = spVectorConstruct(-size.x,-size.y); 
    vertices[1] = spVectorConstruct( size.x,-size.y);
    vertices[2] = spVectorConstruct( size.x, size.y);
    vertices[3] = spVectorConstruct(-size.x, size.y);

    shapes[count].body = spBodyNewDynamic();
    shapes[count].shape = spPolygonNew(vertices, 4, mass);
    shapes[count].shape->material.restitution = 0.2f;
    shapes[count].shape->material.friction = 0.8f;
    spShapeSetFilter(shapes[count].shape, filter);
    spBodySetTransform(shapes[count].body, pos, angle);
    spBodyAddShape(shapes[count].body, shapes[count].shape);
    spWorldAddBody(&demo->world, shapes[count].body);

    shapes[count].color = color;
    shapes[count].border = border;

    count++;

    return count-1;
}

static spInt 
CreateStaticSegment(spVector pointA, spVector pointB, spFloat radius, spColor color, spColor border)
{
    shapes[count].body = spBodyNewStatic();
    shapes[count].shape = spSegmentNew(pointA, pointB, radius, 0.0f);
    spShapeSetFilter(shapes[count].shape, filter);
    spBodyAddShape(shapes[count].body, shapes[count].shape);
    spWorldAddBody(&demo->world, shapes[count].body);
    shapes[count].color = color;
    shapes[count].border = border;

    count++;

    return count-1;
}

static spInt
CreateStaticBox(spVector pos, spFloat angle, spVector size, spColor color, spColor border)
{
    spVector vertices[4];
    vertices[0] = spVectorConstruct(-size.x,-size.y); 
    vertices[1] = spVectorConstruct( size.x,-size.y);
    vertices[2] = spVectorConstruct( size.x, size.y);
    vertices[3] = spVectorConstruct(-size.x, size.y);

    shapes[count].body = spBodyNewStatic();
    shapes[count].shape = spPolygonNew(vertices, 4, 0.0f);
    spShapeSetFilter(shapes[count].shape, filter);
    spBodySetTransform(shapes[count].body, pos, angle);
    spBodyAddShape(shapes[count].body, shapes[count].shape);
    spWorldAddBody(&demo->world, shapes[count].body);

    shapes[count].color = color;
    shapes[count].border = border;

    count++;

    return count-1;
}

static spInt
CreateKinematicBox(spVector pos, spFloat angle, spVector size, spColor color, spColor border)
{
    spVector vertices[4];
    vertices[0] = spVectorConstruct(-size.x,-size.y); 
    vertices[1] = spVectorConstruct( size.x,-size.y);
    vertices[2] = spVectorConstruct( size.x, size.y);
    vertices[3] = spVectorConstruct(-size.x, size.y);

    shapes[count].body = spBodyNewKinematic();
    shapes[count].shape = spPolygonNew(vertices, 4, 0.0f);
    spShapeSetFilter(shapes[count].shape, filter);
    spBodySetTransform(shapes[count].body, pos, angle);
    spBodyAddShape(shapes[count].body, shapes[count].shape);
    spWorldAddBody(&demo->world, shapes[count].body);

    shapes[count].color = color;
    shapes[count].border = border;

    count++;

    return count-1;
}

static spInt
CreateCircle(spVector pos, spFloat radius, spFloat mass, spColor color, spColor border)
{
    shapes[count].body = spBodyNewDynamic();
    shapes[count].shape = spCircleNew(spVectorConstruct(0.0f, 0.0f), radius, mass);
    shapes[count].shape->material.restitution = 0.2f;
    shapes[count].shape->material.friction = 0.8f;
    spBodySetTransform(shapes[count].body, pos, 0.0f);
    spBodyAddShape(shapes[count].body, shapes[count].shape);
    spWorldAddBody(&demo->world, shapes[count].body);

    shapes[count].color = color;
    shapes[count].border = border;

    count++;

    return count-1;
}

static spBody*
Body(spInt i)
{
    return shapes[i].body;
}

static spShape*
Shape(spInt i)
{
    return shapes[i].shape;
}

static void 
setup() 
{
    spSlop = 0.65f;
    spColor lightBlue = RGBA255(176.f, 226.f, 255.f, 0.0f);
    demo->background = lightBlue;

    spConstraint* constraint;
    spInt a, b;

    CreateStaticSegment({ -33.3f, -33.3f}, {-33.3f, 100.0f}, 1.0f, BLACK(), BLACK());
    CreateStaticSegment({  33.3f,-100.0f}, { 33.3f, 100.0f}, 1.0f, BLACK(), BLACK());
    
    CreateStaticSegment({-100.0f,  33.3f}, {100.0f,  33.3f}, 1.0f, BLACK(), BLACK());
    CreateStaticSegment({-100.0f, -33.3f}, {100.0f, -33.3f}, 1.0f, BLACK(), BLACK());

    CreateStaticSegment({-100.0f,-100.0f}, {100.0f,-100.0f}, 1.0f, BLACK(), BLACK());
    CreateStaticSegment({-100.0f, 100.0f}, {100.0f, 100.0f}, 1.0f, BLACK(), BLACK());

    CreateStaticSegment({-100.0f,-100.0f}, {-100.0f,100.0f}, 1.0f, BLACK(), BLACK());
    CreateStaticSegment({ 100.0f,-100.0f}, { 100.0f,100.0f}, 1.0f, BLACK(), BLACK());


    /// *** Motor joint ***
    a = CreateCircle({ 11.6f, 66.6f}, 8.0f, 50.0f, RGB255(0,128,255), BLACK());
    b = CreateCircle({-11.6f, 66.6f}, 8.0f, 50.0f, RGB255(0,128,255), BLACK());
    spBodySetGravityScale(Body(a), 0.0f);
    spBodySetGravityScale(Body(b), 0.0f);
    constraint = spMotorJointNew(Body(a), Body(b), 25.0f);
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Angular spring joint ***
    a = CreateCircle({ 86.6f, 66.6f}, 8.0f, 25.0f, RGB255(0,255,0), BLACK());
    b = CreateCircle({ 46.6f, 66.6f}, 8.0f, 25.0f, RGB255(0,255,0), BLACK());

    spBodySetAngularVelocityDamping(Body(a), 0.0f);
    spBodySetAngularVelocityDamping(Body(b), 0.0f);
    spBodySetGravityScale(Body(a), 0.0f);
    spBodySetGravityScale(Body(b), 0.0f);
    spBodyApplyTorque(Body(a), 40000.0f);

    constraint = spAngularSpringJointNew(Body(a), Body(b), spFalse, 0.5f, 0.1f, 0.0f);
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Gear joint ***
    a = CreateCircle({-86.6f, 66.6f}, 8.0f, 25.0f, RGB255(64,255,0), BLACK());
    b = CreateCircle({-46.6f, 66.6f}, 8.0f, 25.0f, RGB255(64,255,0), BLACK());

    spBodySetAngularVelocityDamping(Body(a), 0.0f);
    spBodySetAngularVelocityDamping(Body(b), 0.0f);
    spBodySetGravityScale(Body(a), 0.0f);
    spBodySetGravityScale(Body(b), 0.0f);
    spBodyApplyTorque(Body(a), 25000.0f);

    constraint = spGearJointNew(Body(a), Body(b), 2.0f, 0.0f);
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Wheel joint ***
    spGroup carGroup = (0<<0);
    spFilter carFilter = spFilterConstruct(carGroup, spCollideAll, spCollideAll);
    spShape* shape = NULL;

    a = CreateBox({-39.6f,-50.0f}, 0.0f, {35.0f, 10.0f}, 75.0f, RGB255(128, 0, 255), BLACK());
    spShapeSetFilter(Shape(a), carFilter);
    b = CreateCircle({-66.6f,-66.6f}, 10.0f, 15.0f, RGB255(255,0,128), BLACK());
    spShapeSetFilter(Shape(b), carFilter);
    constraint = spWheelJointNew(Body(a), Body(b), {-25.0f, -25.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}, 2.0f, 0.7f);
    spWorldAddConstraint(&demo->world, constraint);

    b = CreateCircle({-4.6f,-66.6f}, 10.0f, 15.0f, RGB255(255,0,128), BLACK());
    spShapeSetFilter(Shape(b), carFilter);
    constraint = spWheelJointNew(Body(a), Body(b), { 25.0f, -25.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}, 2.0f, 0.7f);
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Spring joint ***
    a = CreateBox({-75.0f, 0.0f}, 0.0f, {5.0f, 5.0f}, 175.0f, RGB(0.8f, 0.0f, 0.0f), BLACK());
    b = CreateBox({-55.0f, 0.0f}, 0.0f, {5.0f, 5.0f}, 175.0f, RGB(0.8f, 0.0f, 0.0f), BLACK());
    constraint = spSpringJointNew(Body(a), Body(b), spVectorZero(), spVectorZero(), 0.6f, 0.3f, 10.0f);
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Point joint ***
    a = CreateBox({ 10.0f,  0.0f}, 0.0f, {6.0f, 6.0f}, 75.0f, {1.0f, 0.5f, 0.0f, 1.0f}, BLACK());
    b = CreateBox({-10.0f,  0.0f}, 0.0f, {6.0f, 6.0f}, 75.0f, {1.0f, 0.5f, 0.0f, 1.0f}, BLACK());
    constraint = spPointJointNew(Body(a), Body(b), {-8.0f,-8.0f}, {-8.0f,8.0f});
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Rope joint ***
    a = CreateBox({75.0f, 0.0f}, 0.0f, {6.0f, 6.0f}, 75.0f, RGB255(0,128,255), BLACK());
    b = CreateBox({55.0f, 0.0f}, 0.0f, {6.0f, 6.0f}, 75.0f, RGB255(0,128,255), BLACK());
    constraint = spRopeJointNew(Body(a), Body(b), spVectorZero(), spVectorZero(), 25.0f);
    spWorldAddConstraint(&demo->world, constraint);

    /// *** Distance joint ***
    a = CreateBox({ 86.6f,-66.6f}, 0.0f, {7.0f, 7.0f}, 75.0f, RGB(0,0.8,0), BLACK());
    b = CreateBox({ 46.6f,-66.6f}, 0.0f, {7.0f, 7.0f}, 75.0f, RGB(0,0.8,0), BLACK());
    constraint = spDistanceJointNew(Body(a), Body(b), spVectorZero(), spVectorZero(), 20.0f);
    spWorldAddConstraint(&demo->world, constraint);
}

static void update(spFloat dt) 
{ 
    spWorld* world = &demo->world;

    spWorldStep(world, dt);

    for (spInt i = 0; i < count; ++i)
    {
        spDemoDrawShape(shapes[i].shape, shapes[i].color, shapes[i].border);
    }

    spConstraint* constraints = world->jointList;
    spConstraint* constraint = NULL;
    while(constraints != NULL)
    {
        constraint = constraints;
        spDemoDrawConstraint(constraint, GREEN(), GREEN());
        constraints = constraint->next;
    }
}

static void destroy() 
{
}

spDemo* test = spDemoNew(setup, update, destroy);
