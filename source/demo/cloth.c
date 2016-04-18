
#include "demo\spDemo.h"

#define SIZE 16 

typedef spSingleBodyObject wall;
typedef spSingleBodyObject particle;

static particle particles[SIZE*SIZE];
static spConstraint* springs[(SIZE-1)*SIZE*2];
static spPolygon* patch[(SIZE-1)*(SIZE-1)];

static wall walls[4];

/// for easy poly creation
static spVector poly[4] = { 0 };

/// wall positions
static spVector wallPos[8] = {
    { -400, -300 }, {  400, -300 },
    {  400, -300 }, {  400,  300 },
    {  400,  300 }, { -400,  300 },
    { -400,  300 }, { -400, -300 }
};

/// convenience function
static spVector
vec(spFloat x, spFloat y) 
{ 
    return spVectorConstruct(x, y); 
}

static 
spVector* polyNew(spFloat x, spFloat y)
{
    poly[0] = vec(-x, -y);
    poly[1] = vec( x, -y);
    poly[2] = vec( x,  y);
    poly[3] = vec(-x,  y);
    return poly;
}

static void 
Reset()
{
    for (spInt i = 0; i < SIZE; ++i)
    {
        for (spInt j = 0; j < SIZE; ++j)
        {
            spInt index = SIZE * i + j;
            spFloat diff = 400.f / (spFloat)(SIZE-1);
            spDemoSingleBodyTransform(particles + index, vec(-200.f + (j*diff), 275.f - (i*diff)), 0.0f);
        }
    }
}

static void 
Keyboard()
{
    if (spDemoKeyPressed('R')) Reset();
}

static void
Create()
{
    for (spInt i = 0; i < 4; ++i)
    {
        walls[i].shape = spSegmentNew(wallPos[i*2], wallPos[i*2+1], 2.0f, 0.0f);
        walls[i].body = spBodyNewStatic();
        walls[i].color = RGB(0,0,0);
        walls[i].border = RGB(0,0,0);
        spBodyAddShape(walls[i].body, walls[i].shape);
        spWorldAddBody(&Demo->world, walls[i].body);
    }

    spGroup clothGroup = 1;
    spFilter clothFilter = spFilterConstruct(clothGroup, spCollideNone, spCollideNone);
    for (spInt i = 0; i < SIZE; ++i)
    {
        for (spInt j = 0; j < SIZE; ++j)
        {
            int index = SIZE * i + j;
            particles[index].shape = spCircleNew(vec(0,0), (400.f/(spFloat)SIZE), 100000.0f);
            particles[index].body = (i == 0) ? spBodyNewStatic() : spBodyNewDynamic();
            particles[index].color = spDemoRandomPastelColor();
            particles[index].border = RGB(0,0,0);
            particles[index].shape->filter = clothFilter;
            spBodySetLinearVelocityDamping(particles[index].body, 0.9f);
            spBodySetAngularVelocityDamping(particles[index].body, 5.0f);
            spBodyAddShape(particles[index].body, particles[index].shape);
            spWorldAddBody(&Demo->world, particles[index].body);
        }
    }

    for (spInt i = 0; i < (SIZE-1)*(SIZE-1); ++i)
    {
        patch[i] = spPolygonNew(polyNew(1, 1), 4, 1.f);
    }

    spFloat f = 4.0f;
    spFloat d = 1.0f;
    spFloat r = 400.f / ((spFloat)SIZE - 1.f);

    int p = 0;
    for (spInt i = 0; i <SIZE*(SIZE-1); i+=(SIZE-1))
    {
        for (spInt j = 0; j < (SIZE-1); ++j)
        {
            springs[i+j]  = spSpringJointNew(particles[p+j].body, particles[p+j+1].body, vec(0,0), vec(0,0), f, d, r);
        }
        p += SIZE;
    }

    p = 0;
    for (spInt i = 0; i < SIZE*(SIZE-1); i+=SIZE)
    {
        for (spInt j = 0; j < SIZE; j++)
        {
            springs[i+j+(SIZE*(SIZE-1))] = spSpringJointNew(particles[p+j].body, particles[p+j+SIZE].body, vec(0, 0), vec(0, 0), f, d, r);
        }
        p += SIZE;
    }

    for (spInt i = 0; i < SIZE*(SIZE-1)*2; ++i)
    {
        spWorldAddConstraint(&Demo->world, springs[i]);
    }
}

static void 
Setup()
{
    spDemoInitRandomSeed();
    spDemoSetKeyCallback(Keyboard);

    Demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    Demo->world.gravity = spvfMult(Demo->world.gravity, 1.5f);

    spBaumgarte = 0.07f;
    spLineScaleSmall = 3.f;
    spLineScaleBig = 0.5f;
    spSlop = 0.65f;

    Create();
    Reset();
}

static void
TransformPatch(spPolygon* poly, spInt start)
{
    spVector v0 = poly->edges[0].vertex = spBodyGetPosition(particles[start+0].body);
    spVector v1 = poly->edges[1].vertex = spBodyGetPosition(particles[start+1].body);
    spVector v2 = poly->edges[2].vertex = spBodyGetPosition(particles[start+SIZE+1].body);
    spVector v3 = poly->edges[3].vertex = spBodyGetPosition(particles[start+SIZE].body);
    spVector com = spvAdd(spvAdd(v0, v1), spvAdd(v2, v3));
    poly->shape.mass_data.com = spVectorConstruct(com.x/4.f, com.y/4.f);
}

static void
Render()
{
    int p = 0;
    for (spInt i = 0; i < (SIZE-1)*(SIZE-1); i+=(SIZE-1))
    {
        for (spInt j = 0; j < (SIZE-1); j++)
        {
            TransformPatch(patch[i+j], p+j);
        }
        p += SIZE;
    }

    for (spInt i = 0; i < (SIZE-1)*(SIZE-1); ++i)
    {
        spColor c = RGB(0.6,0,0);
        if ((i % 2) == 0) c = RGB(0.8,0,0);
        spDemoDrawPolygon(patch[i], c, c);
    }

    for (spInt i = 0; i < 4; ++i)
    {
        spDemoDrawSingleBody(walls+i);
    }

    for (spInt i = 0; i < SIZE*(SIZE-1)*2; ++i)
    {
        spDemoDrawConstraint(springs[i], RGB(0,0,0), RGB(0,0,0));
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
    spWorldDestroy(&Demo->world);
}

static void 
Run(void)
{
    spDemoRun(spDemoNew(Setup, Update, Render, Destroy, spFrustumView(400, 300), spViewportNew(800, 600)));
}

/// extern function pointer
Cloth = Run;