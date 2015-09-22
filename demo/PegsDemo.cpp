
#include "spDemo.h"
#include "spCircle.h"
#include "spPolygon.h"
#include "spSegment.h"

typedef spSingleBodyObject peg;
typedef spSingleBodyObject ball;

static const spInt pegCount = 33;
static peg pegs[pegCount];

static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
CreatePegs()
{
    for (spInt i = 0; i < pegCount; ++i)
    {
        pegs[i].body = spBodyNewStatic();
        pegs[i].shape = spCircleNew(vec(0.f, 0.f), 25.f, 0.f);
        pegs[i].color = RGB(0,0,0); pegs[i].border = RGB(0,0,0);
        spBodyAddShape(pegs[i].body, pegs[i].shape);
        spWorldAddBody(&demo->world, pegs[i].body);
    }


    spFloat x = -500.f;
    spFloat y =  300.f;
    for (spInt i = 0; i < 6; ++i)
    {
        spBodySetTransform(pegs[i].body, vec(x, y), 0.f);
        x += 200.f;
    }

    x = -400.f;
    y =  100.f;
    for (spInt i = 6; i < 11; ++i)
    {
        spBodySetTransform(pegs[i].body, vec(x, y), 0.f);
        x += 200.f;
    }

    x = -500.f;
    y = -100.f;
    for (spInt i = 11; i < 17; ++i)
    {
        spBodySetTransform(pegs[i].body, vec(x, y), 0.f);
        x += 200.f;
    }

    x = -400.f;
    y = -300.f;
    for (spInt i = 17; i < 22; ++i)
    {
        spBodySetTransform(pegs[i].body, vec(x, y), 0.f);
        x += 200.f;
    }

    x = -500.f;
    y = -500.f;
    for (spInt i = 22; i < 28; ++i)
    {
        spBodySetTransform(pegs[i].body, vec(x, y), 0.f);
        x += 200.f;
    }

    x = -400.f;
    y = -700.f;
    for (spInt i = 28; i < 33; ++i)
    {
        spBodySetTransform(pegs[i].body, vec(x, y), 0.f);
        x += 200.f;
    }
}

static void
Create()
{
    CreatePegs();
}

static void
Reset()
{
}

static void
Keyboard()
{
    if (spDemoKeyPressed('r') || spDemoKeyPressed('R')) Reset();
}

static void
Setup()
{
    spSlop = 2.0f;
    spLineScaleSmall = 8.0f;
    spLineScaleBig = 14.0f;

    demo->background = RGBA255(176.f, 226.f, 255.f, 0.0f);
    demo->keyboard = Keyboard;

    spDemoInitRandomSeed();
    Create();
    Reset();
}

static void
Render()
{
    for (spInt i = 0; i < pegCount; ++i)
    {
        spDemoDrawShape(pegs[i].shape, pegs[i].color, pegs[i].border);
    }
}

static void 
Update(spFloat dt)
{
    spWorldStep(&demo->world, dt);
    Render();
}

static void 
Destroy()
{
}

spDemo* Pegs = spDemoNew(Setup, Update, Destroy, spFrustumView(600, 800), { 600, 800 });