
#include "demo\spDemo.h"

#define LINKS 12
#define BALLS 8
#define BOXES 8

typedef spSingleBodyObject wall;
typedef spSingleBodyObject link;
typedef spSingleBodyObject ball;
typedef spSingleBodyObject box;

static wall walls[4];
static link links[LINKS];
static ball balls[BALLS];
static box boxes[BALLS];
static spConstraint* chains[LINKS];

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
    spVector position = vec(-200, 100);
    for (spInt i = 0; i < LINKS/2; ++i)
    {
        spDemoSingleBodyTransform(links+i, spvAdd(position, vec(0, -50.f*(spFloat)i)), 0.0f);
    }

    position = vec(200, 100);
    for (spInt i = LINKS/2; i < LINKS; ++i)
    {
        spDemoSingleBodyTransform(links+i, spvAdd(position, vec(0, -50.f*((spFloat)i-6.f))), 0.0f);
    }

    for (spInt i = 0; i < BALLS; ++i)
    {
        spDemoSingleBodyTransform(balls+i, vec(spDemoRandomFloatRange(-300.0f, 300.0f), spDemoRandomFloatRange( 200,  100)), 0.0f);
    }

    for (spInt i = 0; i < BOXES; ++i)
    {
        spDemoSingleBodyTransform(boxes+i, vec(spDemoRandomFloatRange(-300.0f, 300.0f), spDemoRandomFloatRange( 200,  100)), spDemoRandomFloatRange(0.f, 90.f));
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

    for (spInt i = 0; i < BALLS; ++i)
    {
        balls[i].shape = spCircleNew(vec(0,0), spDemoRandomFloatRange(15.0f, 50.0f), 500.0f);
        balls[i].body = spBodyNewDynamic();
        balls[i].color = spDemoRandomPastelColor();
        balls[i].border = RGB(0,0,0);
        spBodyAddShape(balls[i].body, balls[i].shape);
        spWorldAddBody(&Demo->world, balls[i].body);
    }

    for (spInt i = 0; i < BALLS; ++i)
    {
        boxes[i].shape = spPolygonNew(polyNew(spDemoRandomFloatRange(20.0f, 50.0f), spDemoRandomFloatRange(10.0f, 50.0f)), 4, 500.0f);
        boxes[i].body = spBodyNewDynamic();
        boxes[i].color = spDemoRandomPastelColor();
        boxes[i].border = RGB(0,0,0);
        spBodyAddShape(boxes[i].body, boxes[i].shape);
        spWorldAddBody(&Demo->world, boxes[i].body);
    }

    spGroup chainGroup = 1;
    spFilter chainFilter = spFilterConstruct(chainGroup, spCollideAll, spCollideAll);
    for (spInt i = 0; i < LINKS; ++i)
    {
        links[i].shape = spPolygonNew(polyNew(8, 22), 4, 400.0f);
        links[i].body = spBodyNewDynamic();
        links[i].color = spDemoRandomPastelColor();
        links[i].border = RGB(0,0,0);
        spShapeSetMaterial(links[i].shape, 0.1f, 0.9f);
        spBodySetLinearVelocityDamping(links[i].body, 0.75f);
        spBodySetAngularVelocityDamping(links[i].body, 0.75f);
        spBodyAddShape(links[i].body, links[i].shape);
        spWorldAddBody(&Demo->world, links[i].body);
        spShapeSetFilter(links[i].shape, chainFilter);
    }

    for (spInt i = 1; i < LINKS/2; ++i)
    {
        chains[i] = spDistanceJointNew(links[i-1].body, links[i].body, vec(0,-22), vec(0, 24), 0.0f);
        spWorldAddConstraint(&Demo->world, chains[i]);
    }

    for (spInt i = LINKS/2+1; i < LINKS; ++i)
    {
        chains[i] = spDistanceJointNew(links[i-1].body, links[i].body, vec(0,-22), vec(0, 24), 0.0f);
        spWorldAddConstraint(&Demo->world, chains[i]);
    }

    chains[0] = spDistanceJointNew(walls[0].body, links[0].body, vec(-200, 100), vec(0, 22), 1.0f);
    chains[6] = spDistanceJointNew(walls[0].body, links[6].body, vec( 200, 100), vec(0, 22), 1.0f);
    spWorldAddConstraint(&Demo->world, chains[0]);
    spWorldAddConstraint(&Demo->world, chains[6]);

}

static void 
Setup()
{
    spDemoInitRandomSeed();
    spDemoSetKeyCallback(Keyboard);

    Demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    Demo->world.gravity = spvfMult(Demo->world.gravity, 1.5f);

    spBaumgarte = 0.07f;
    spLineScaleSmall = 5.f;
    spLineScaleBig = 10.0f;
    spSlop = 0.65f;

    Create();
    Reset();
}

static void
Render()
{
    for (spInt i = 0; i < 4; ++i)
    {
        spDemoDrawSingleBody(walls+i);
    }

    for (spInt i = 0; i < BALLS; ++i)
    {
        spDemoDrawSingleBody(balls+i);
    }

    for (spInt i = 0; i < BOXES; ++i)
    {
        spDemoDrawSingleBody(boxes+i);
    }

    for (spInt i = 0; i < LINKS; ++i)
    {
        spDemoDrawSingleBody(links+i);
    }

    for (spInt i = 0; i < LINKS; ++i)
    {
        spDemoDrawConstraint(chains[i], RGB(1,0,0), RGB(1,0,0));
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
Chains = Run;