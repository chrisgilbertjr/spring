
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

static void
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

static void 
setup() 
{
    spSlop = 0.65f;

    spColor color  = {0.4f, 0.8f, 0.8f, 1.0f};
    spColor border = {1.0f, 1.0f, 1.0f, 1.0f};

    CreateStaticBox({0.0f, -100.0f}, 0.0f, {200.0f, 5.0f}, color, border);
    CreateStaticBox({-100.0f, 0.0f}, 0.0f, {5.0f, 200.0f}, color, border);
    CreateStaticBox({ 100.0f, 0.0f}, 0.0f, {5.0f, 200.0f}, color, border);

    spConstraint* constraint;
    spInt a, b;

    //CreateCircle({-25.0f, 25.0f},  8.0f, 50.0f, RED(), WHITE());
    //CreateCircle({  9.0f, 10.0f}, 10.0f, 50.0f, RED(), WHITE());
    //CreateCircle({ 60.0f, 25.0f}, 12.0f, 50.0f, RED(), WHITE());
    //CreateCircle({-30.0f, 25.0f},  8.0f, 50.0f, RED(), WHITE());
    //CreateCircle({ 10.0f, 10.0f}, 10.0f, 50.0f, RED(), WHITE());
    //CreateCircle({ 90.0f, 25.0f}, 12.0f, 50.0f, RED(), WHITE());
    //CreateCircle({-20.0f, 35.0f},  8.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({  0.0f, 90.0f}, 10.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({ 20.0f, 15.0f}, 12.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({-20.0f, 25.0f},  8.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({  0.0f, 40.0f}, 10.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({ 20.0f, 35.0f}, 12.0f, 50.0f, YELLOW(), WHITE());

    //CreateBox({-10.0f,-10.0f}, 0.0f, {6.0f, 6.0f}, 100.0f, BLUE(), WHITE());
    //CreateBox({ 10.0f,-10.0f}, 1.0f, {7.0f, 6.0f}, 100.0f, BLUE(), WHITE());
    //CreateBox({ 10.0f, 10.0f}, 2.0f, {8.0f, 6.0f}, 100.0f, BLUE(), WHITE());
    //CreateBox({-10.0f, 10.0f}, 3.0f, {9.0f, 6.0f}, 100.0f, BLUE(), WHITE());

    //CreateBox({-40.0f,-40.0f}, 0.0f, {12.0f, 10.0f}, 200.0f, PURPLE(), WHITE());
    //CreateBox({ 40.0f,-40.0f}, 1.0f, {11.0f, 10.0f}, 200.0f, PURPLE(), WHITE());
    //CreateBox({ 40.0f, 40.0f}, 2.0f, {10.0f, 10.0f}, 200.0f, PURPLE(), WHITE());
    //CreateBox({-40.0f, 40.0f}, 3.0f, { 9.0f, 10.0f}, 200.0f, PURPLE(), WHITE());

    //CreateBox({-50.0f,-40.0f}, 0.0f, {12.0f, 10.0f}, 75.0f, BLUE(), WHITE());
    //CreateBox({ 50.0f,-40.0f}, 1.0f, {11.0f, 10.0f}, 75.0f, BLUE(), WHITE());
    //CreateBox({ 50.0f, 40.0f}, 2.0f, {10.0f, 10.0f}, 75.0f, BLUE(), WHITE());
    //CreateBox({-50.0f, 40.0f}, 3.0f, { 9.0f, 10.0f}, 75.0f, BLUE(), WHITE());

    //CreateBox({-60.0f,-40.0f}, 0.0f, {12.0f, 10.0f}, 75.0f, GREEN(), WHITE());
    //CreateBox({ 60.0f,-40.0f}, 1.0f, {11.0f, 10.0f}, 75.0f, GREEN(), WHITE());
    //CreateBox({ 60.0f, 40.0f}, 2.0f, {10.0f, 10.0f}, 75.0f, GREEN(), WHITE());
    //CreateBox({-60.0f, 40.0f}, 3.0f, { 9.0f, 10.0f}, 75.0f, GREEN(), WHITE());

    //CreateCircle({  9.0f, 10.0f}, 10.0f, 50.0f, RED(), WHITE());
    //CreateCircle({ 60.0f, 25.0f}, 12.0f, 50.0f, RED(), WHITE());
    //CreateCircle({-30.0f, 25.0f},  8.0f, 50.0f, RED(), WHITE());
    //CreateCircle({ 10.0f, 10.0f}, 10.0f, 50.0f, RED(), WHITE());
    //CreateCircle({ 90.0f, 25.0f}, 12.0f, 50.0f, RED(), WHITE());
    //CreateCircle({-20.0f, 35.0f},  8.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({  0.0f, 90.0f}, 10.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({ 20.0f, 15.0f}, 12.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({-20.0f, 25.0f},  8.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({  0.0f, 40.0f}, 10.0f, 50.0f, YELLOW(), WHITE());
    //CreateCircle({ 20.0f, 35.0f}, 12.0f, 50.0f, YELLOW(), WHITE());

    //CreateBox({-10.0f,-10.0f}, 0.0f, {6.0f, 6.0f}, 100.0f, BLUE(), WHITE());
    //CreateBox({ 10.0f,-10.0f}, 1.0f, {7.0f, 6.0f}, 100.0f, BLUE(), WHITE());
    //CreateBox({ 10.0f, 10.0f}, 2.0f, {8.0f, 6.0f}, 100.0f, BLUE(), WHITE());
    //CreateBox({-10.0f, 10.0f}, 3.0f, {9.0f, 6.0f}, 100.0f, BLUE(), WHITE());

    a = CreateBox({0.0f,0.0f}, 0.0f, {35.0f, 10.0f}, 400.0f, PURPLE(), WHITE());
    b = CreateCircle({-25.0f,-40.0f}, 10.0f, 15.0f, RED(), WHITE());
    constraint = spWheelJointNew(Body(a), Body(b), {-25.0f, -40.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}, 2.0f, 0.7f);
    spWorldAddConstraint(&demo->world, constraint);

    b = CreateCircle({ 25.0f,-40.0f}, 10.0f, 15.0f, RED(), WHITE());
    constraint = spWheelJointNew(Body(a), Body(b), { 25.0f, -40.0f}, {0.0f, 0.0f}, {0.0f, 1.0f}, 2.0f, 0.7f);
    spWorldAddConstraint(&demo->world, constraint);

    a = CreateBox({-50.0f,-40.0f}, 0.0f, {5.0f, 5.0f}, 175.0f, RGB(0.2f, 0.0f, 0.8f), WHITE());
    b = CreateBox({ 50.0f,-40.0f}, 1.0f, {5.0f, 5.0f}, 175.0f, RGB(0.2f, 0.0f, 0.8f), WHITE());
    constraint = spSpringJointNew(shapes[a].body, shapes[b].body, spVectorZero(), spVectorZero(), 0.8f, 0.0f, 40.0f);
    spWorldAddConstraint(&demo->world, constraint);

    a = CreateBox({ 25.0f,  0.0f}, 2.0f, {10.0f, 10.0f}, 75.0f, {1.0f, 0.5f, 0.0f, 1.0f}, WHITE());
    b = CreateBox({-25.0f,  0.0f}, 3.0f, {10.0f, 10.0f}, 75.0f, {1.0f, 0.5f, 0.0f, 1.0f}, WHITE());
    constraint = spPointJointNew(shapes[a].body, shapes[b].body, {-15.0f,-15.0f}, {-15.0f,15.0f});
    spWorldAddConstraint(&demo->world, constraint);

    a = CreateBox({-60.0f,-40.0f}, 0.0f, {12.0f, 10.0f}, 75.0f, GREEN(), WHITE());
    b = CreateBox({ 60.0f,-40.0f}, 1.0f, {11.0f, 10.0f}, 75.0f, GREEN(), WHITE());
    constraint = spRopeJointNew(shapes[a].body, shapes[b].body, spVectorZero(), spVectorZero(), 50.0f);
    spWorldAddConstraint(&demo->world, constraint);

    a = CreateBox({ 60.0f, 40.0f}, 2.0f, {10.0f, 10.0f}, 75.0f, RGB(0.2,0.2,0.5), WHITE());
    b = CreateBox({-60.0f, 40.0f}, 3.0f, { 9.0f, 10.0f}, 75.0f, RGB(0.2,0.2,0.5), WHITE());
    constraint = spDistanceJointNew(shapes[a].body, shapes[b].body, spVectorZero(), spVectorZero(), 50.0f);
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
