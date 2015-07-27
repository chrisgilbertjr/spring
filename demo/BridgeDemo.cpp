
#include "spDemoFactory.h"

static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
Dampen(spInt i)
{
    Body(i)->vDamp = 1.0f;
    Body(i)->wDamp = 1.0f;
}

static void
setup()
{
    spSlop = 2.0f;
    spBaumgarte = 0.001f;
    spColor lightBlue = RGBA255(176.f, 226.f, 255.f, 0.0f);
    demo->background = lightBlue;
    spLineScaleSmall = 8.0f;
    spLineScaleBig = 16.0f;
    spBaumgarte = 0.1f;

    spConstraint* constraint;
    spInt a, b;
    spFloat mass = 200.0f;
    spColor color = RGB(0.0f, 0.5f, 1.0f);
    spColor border = RGB(0,0,0);

    spInt s0 = spCreateStaticSegment(vec(0.0f, 0.0f), 0.0f, vec(-1000.0f, -1000.0f), vec(-1000.0f,1000.0f), 10.0f, 250.0f, RGB(1,0,0), RGB(0,0,0));
    spInt s1 = spCreateStaticSegment(vec(0.0f, 0.0f), 0.0f, vec( 1000.0f, -1000.0f), vec( 1000.0f,1000.0f), 10.0f, 250.0f, RGB(1,0,0), RGB(0,0,0));

    spCreateDemoCircle(vec(0.0f, 100.0f), 100.0f, 200.0f, color, border, SP_BODY_DYNAMIC);

    spVector size = vec(30,20);
    spFloat dist = 15.0f;

    spInt iters = 12;
    spFloat start = -900.0f;
    spFloat inc = 1800.0f / (spFloat)iters;

    a = spCreateBox(vec(-900,0), 0, size, mass, color, border);
    spAddConstraint(spRopeJointNew(Body(s0), Body(a), vec(-1000, 250), vec(-size.x, 0), 50.0f));
    Dampen(a);

    for (spInt i = 0 ; i < iters; ++i)
    {
        spVector pos = vec(start + (i*inc), 0);

        b = spCreateBox(pos, 0, size, mass, color, border);
        spAddConstraint(spRopeJointNew(Body(a), Body(b), vec(size.x, 0), vec(-size.x, 0), dist));
        Dampen(b);

        pos.x += inc;
        a = spCreateBox(pos, 0, size, mass, color, border);
        spAddConstraint(spRopeJointNew(Body(b), Body(a), vec(size.x, 0), vec(-size.x, 0), dist));
        Dampen(a);
    }

    b = spCreateBox(vec( 900,0), 0, size, mass, color, border);
    spAddConstraint(spRopeJointNew(Body(a), Body(b), vec(size.x, 0), vec(-size.x, 0), dist));
    Dampen(b);

    spAddConstraint(spRopeJointNew(Body(s1), Body(b), vec(1000, 250), vec(size.x, 0), 50.0f));
}

static void update(spFloat dt)
{
    spWorldStep(&demo->world, dt);
    spDrawFactoryObjects();
}

static void destroy()
{
}

spDemo* bridge = spDemoNew(setup, update, destroy, spFrustumView(1366, 768), { 1366, 768 });