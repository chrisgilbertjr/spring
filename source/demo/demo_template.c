
#include "demo\spDemo.h"

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
}

static void
Create()
{
}

static void 
Setup()
{
    Demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    Demo->world.gravity = vec(0,0);

    spLineScaleSmall = 1.5f;
    spLineScaleBig = 3.0f;
    spSlop = 0.65f;

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
    spWorldStep(&Demo->world, dt);
    Render();
}

static void 
Destroy()
{
}

int main(void)
{
    spDemoRun(spDemoNew(Setup, Update, Render, Destroy, spFrustumView(100, 100), spViewportNew(1366, 768)));
    return 0;
}