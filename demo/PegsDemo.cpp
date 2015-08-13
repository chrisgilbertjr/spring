
#include "spDemo.h"
#include "spPolygon.h"
#include "spSegment.h"

typedef spSingleBodyObject peg;
typedef spSingleBodyObject ball;

static peg pegs[6];

static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
Create()
{
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

spDemo* pegs = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });