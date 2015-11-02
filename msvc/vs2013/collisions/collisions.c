
#include "demo\spDemo.h"

typedef spSingleBodyObject ball;
typedef spSingleBodyObject wall;

#define BOXES 32
ball balls[BOXES];
wall walls[4];

/// convenience function
static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
Keyboard()
{
}

static void
Reset()
{
    spDemoInitRandomSeed();
    for (spInt i = 0; i < BOXES; ++i)
    {
        spFloat x = spDemoRandomFloatRange(-80, 80);
        spFloat y = spDemoRandomFloatRange(-80, 80);
        spBodySetTransform(balls[i].body, vec(x, y), 0.0f);
    }
}

static void
Create()
{
    /// create the ball objects
    for (spInt i = 0; i < BOXES; ++i)
    {
        balls[i].body = spBodyNew(SP_BODY_DYNAMIC);
        balls[i].border = RGB(0,0,0);
        balls[i].color = spDemoRandomPastelColor();
        balls[i].shape = spCircleNew(vec(0,0), spDemoRandomFloatRange(10, 20), spDemoRandomFloatRange(5.0f, 10.0f));
        spBodyAddShape(balls[i].body, balls[i].shape);
        spWorldAddBody(&Demo->world, balls[i].body);
    }

    /// create the wall segments
    walls[0].shape = spSegmentNew(vec(-100.0f, -100.0f), vec( 100.0f, -100.0f), 1.0f, 0.0f);
    walls[1].shape = spSegmentNew(vec(-100.0f,  100.0f), vec( 100.0f,  100.0f), 1.0f, 0.0f);
    walls[2].shape = spSegmentNew(vec(-100.0f, -100.0f), vec(-100.0f,  100.0f), 1.0f, 0.0f);
    walls[3].shape = spSegmentNew(vec( 100.0f, -100.0f), vec( 100.0f,  100.0f), 1.0f, 0.0f);

    /// initialize the single body objects
    for (spInt i = 0; i < 4; ++i)
    {
        walls[i].body = spBodyNewStatic();
        walls[i].color  = RGB(0,0,0);
        walls[i].border = RGB(0,0,0);

        spBodyAddShape(walls[i].body, walls[i].shape);
        spWorldAddBody(&Demo->world, walls[i].body);
    }
}

static void
Setup()
{
    Demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    Demo->world.gravity = vec(0,0);

    spLineScaleSmall = 1.0f;
    spLineScaleBig = 1.5f;
    spSlop = 0.25f;

    Create();
    Reset();
}

static void
Render()
{
    for (spInt i = 0; i < BOXES; ++i)
    {
        spDemoDrawSingleBody(balls+i);
    }
    for (spInt i = 0; i < 4; ++i)
    {
        spDemoDrawSingleBody(walls+i);
    }
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
    //for (spInt i = 0; i < 1; ++i)
    //{
    //    spCircleFree(&balls[i].shape);
    //    spBodyFree(&balls[i].body);
    //}
}

int main(void)
{
    spDemoRun(spDemoNew(Setup, Update, Render, Destroy, spFrustumView(100, 100), spViewportNew(800, 800)));
    return 0;
}