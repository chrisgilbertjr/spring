
#include "demo\spDemo.h"

typedef spSingleBodyObject ball;
typedef spSingleBodyObject wall;
typedef spMultiBodyObject multi;

#define BALLS 64

static wall walls[4];
static ball balls[BALLS];
static multi convex[2];

/// for easy poly creation
spVector poly[4] = { 0 };

/// convenience functions
static spVector
vec(spFloat x, spFloat y) 
{ 
    return spVectorConstruct(x, y); 
}

static 
spVector* polyNew(spFloat x, spFloat y, spVector offset)
{
    poly[0] = spvAdd(offset, vec(-x, -y));
    poly[1] = spvAdd(offset, vec( x, -y));
    poly[2] = spvAdd(offset, vec( x,  y));
    poly[3] = spvAdd(offset, vec(-x,  y));
    return poly;
}

static void 
Reset()
{
    /// reset the positions and clear all forces
    spFloat x = -80.0f;
    for (spInt i = 0; i < BALLS; ++i)
    {
        spFloat y = spDemoRandomFloatRange(-20.0f, 80.0f);
        spDemoSingleBodyTransform(balls+i, vec(x, y), 0);
        x += 5.0f;
    }

    spBodySetTransform(convex[0].body, vec(-50,-60), 0);
    spBodyClearForces(convex[0].body);
    spBodyClearVelocity(convex[0].body);

    spBodySetTransform(convex[1].body, vec( 50,-60), 0);
    spBodyClearForces(convex[1].body);
    spBodyClearVelocity(convex[1].body);
}

static void 
Keyboard()
{
    if (spDemoKeyPressed('R')) Reset();
    if (spDemoKeyPressed('G'))
    {
        if (spvEqual(Demo->world.gravity, vec(0, 0)))
        {
            Demo->world.gravity = spDemoGravity();
        }
        else
        {
            Demo->world.gravity = vec(0, 0);
        }
    }
}

static void
Create()
{
    /// create the walls
    walls[0].shape = spSegmentNew(vec(-100,-100), vec( 100,-100), 2.0f, 0.0f);
    walls[1].shape = spSegmentNew(vec( 100,-100), vec( 100, 100), 2.0f, 0.0f);
    walls[2].shape = spSegmentNew(vec( 100, 100), vec(-100, 100), 2.0f, 0.0f);
    walls[3].shape = spSegmentNew(vec(-100, 100), vec(-100,-100), 2.0f, 0.0f);
    for (spInt i = 0; i < 4; i++)
    {
        walls[i].body = spBodyNewStatic();
        walls[i].color = RGB(0,0,0);
        walls[i].border = RGB(0,0,0);
        spBodyAddShape(walls[i].body, walls[i].shape);
        spWorldAddBody(&Demo->world, walls[i].body);
    }

    /// create the balls
    for (spInt i = 0; i < BALLS; ++i)
    {
        balls[i].body = spBodyNewDynamic();
        balls[i].color = spDemoRandomScaleColor(0.5f);
        balls[i].border = RGB(0,0,0);
        balls[i].shape = spCircleNew(vec(0,0), spDemoRandomFloatRange(5.0f, 15.0f), 20.f);
        spBodyAddShape(balls[i].body, balls[i].shape);
        spWorldAddBody(&Demo->world, balls[i].body);
    }

    /// create convex objects
    for (spInt i = 0; i < 2; i++)
    {
        spColor bColor = RGB(0,0,0);

        spMultiBodyObjectInit(convex+i, 5);
        convex[i].body = spBodyNewDynamic();

        convex[i].shapes[0] = spPolygonNew(polyNew(10, 3, vec(-25,0)), 4, 25.f);
        convex[i].borders[0] = RGB(0,0,0);
        convex[i].colors[0] = bColor;

        convex[i].shapes[1] = spPolygonNew(polyNew(10, 3, vec(25,0)), 4, 25.f);
        convex[i].borders[1] = RGB(0,0,0);
        convex[i].colors[1] = bColor;

        convex[i].shapes[2] = spPolygonNew(polyNew(3, 10, vec(0,-25)), 4, 25.f);
        convex[i].borders[2] = RGB(0,0,0);
        convex[i].colors[2] = bColor;

        convex[i].shapes[3] = spPolygonNew(polyNew(3, 10, vec(0, 25)), 4, 25.f);
        convex[i].borders[3] = RGB(0,0,0);
        convex[i].colors[3] = bColor;

        convex[i].shapes[4] = spCircleNew(vec(0,0), 15, 50.0f);
        convex[i].borders[4] = RGB(0,0,0);
        convex[i].colors[4] = bColor;

        spBodyAddShape(convex[i].body, convex[i].shapes[0]);
        spBodyAddShape(convex[i].body, convex[i].shapes[1]);
        spBodyAddShape(convex[i].body, convex[i].shapes[2]);
        spBodyAddShape(convex[i].body, convex[i].shapes[3]);
        spBodyAddShape(convex[i].body, convex[i].shapes[4]);
        spWorldAddBody(&Demo->world, convex[i].body);
    }
}

static void 
Setup()
{
    spDemoSetKeyCallback(Keyboard);
    Demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    Demo->world.iterations = 6;
    spDemoInitRandomSeed();

    spLineScaleSmall = 1.5f;
    spLineScaleBig = 3.0f;
    spSlop = 0.65f;

    Create();
    Reset();
}

static void
Render()
{
    for (spInt i = 0; i < 4; i++)
    {
        spDemoDrawSingleBody(walls+i);
    }

    for (spInt i = 0; i < BALLS; i++)
    {
        spDemoDrawSingleBody(balls+i);
    }

    spDemoDrawMultiBody(convex+0);
    spDemoDrawMultiBody(convex+1);

    spDemoDrawMouse();
}

static void
Update(spFloat dt)
{
    spWorldStep(&Demo->world, dt);
}

static void 
Destroy()
{
    spWorldDestroy(&Demo->world);
}

int main(void)
{
    spDemoRun(spDemoNew(Setup, Update, Render, Destroy, spFrustumView(100, 100), spViewportNew(800, 800)));
    return 0;
}
