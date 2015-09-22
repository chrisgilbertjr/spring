
#include "spDemo.h"
#include "spPolygon.h"
#include "spSegment.h"
#include "spCircle.h"

typedef spSingleBodyObject spBox;
typedef spSingleBodyObject spBall;
typedef spSingleBodyObject spWall;

static spConstraint joints[9];
static spConstraint pins[6];
static spBox  boxes[13];
static spBall balls[4];
static spWall walls[8];

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
CreateWalls()
{
    /// create the wall segments
    walls[0].shape = spSegmentNew(vec( -33.3f,  -33.3f), vec( -33.3f,  100.0f), 1.0f, 0.0f);
    walls[1].shape = spSegmentNew(vec(  33.3f, -100.0f), vec(  33.3f,  100.0f), 1.0f, 0.0f);
    walls[2].shape = spSegmentNew(vec(-100.0f,   33.3f), vec( 100.0f,   33.3f), 1.0f, 0.0f);
    walls[3].shape = spSegmentNew(vec(-100.0f,  -33.3f), vec( 100.0f,  -33.3f), 1.0f, 0.0f);
    walls[4].shape = spSegmentNew(vec(-100.0f, -100.0f), vec( 100.0f, -100.0f), 1.0f, 0.0f);
    walls[5].shape = spSegmentNew(vec(-100.0f,  100.0f), vec( 100.0f,  100.0f), 1.0f, 0.0f);
    walls[6].shape = spSegmentNew(vec(-100.0f, -100.0f), vec(-100.0f,  100.0f), 1.0f, 0.0f);
    walls[7].shape = spSegmentNew(vec( 100.0f, -100.0f), vec( 100.0f,  100.0f), 1.0f, 0.0f);

    /// initialize the single body objects
    for (spInt i = 0; i < 8; ++i)
    {
        walls[i].body = spBodyNewStatic();
        walls[i].color  = RGB(0,0,0);
        walls[i].border = RGB(0,0,0);

        spBodyAddShape(walls[i].body, walls[i].shape);
        spWorldAddBody(&demo->world, walls[i].body);
    }
}

static void
CreateConstraints()
{
}

static void
CreateShapes()
{
    /// create the balls
    {
        spVector positions[4] = { vec(-72.6f, 66.6f), vec(-46.6f, 66.6f), vec(-66.6f, -66.6f), vec(-4.6f, -66.6f) };
        spColor colors[4] = { RGB(0.25,1,0), RGB(0.25,1,0), RGB(0.5,0,1), RGB(0.5,0,1) };
        spFloat radius[4] = { 18.0f, 8.0f, 10.0f , 10.0f };
        spFloat mass[4] = { 25.0f, 25.0f, 15.0f, 15.0f };

        for (spInt i = 0; i < 4; ++i)
        {
            balls[i].body = spBodyNewDynamic();
            balls[i].shape = spCircleNew(vec(0,0), radius[i], mass[i]);
            balls[i].color = colors[i];
            balls[i].border = RGB(0,0,0);

            spShapeSetMaterial(balls[i].shape, 0.2f, 0.8f);
            spBodyAddShape(balls[i].body, balls[i].shape);
            spWorldAddBody(&demo->world, balls[i].body);
        }
    }

    /// create the boxes
    {
        spColor colors[13] = 
        {
            RGB(0,0,0), RGB(0,0,0),
            RGB(0,0,0), RGB(0,0,0),
            RGB(0,0,0), RGB(0,0,0),
            RGB(0,0,0), RGB(0,0,0),
            RGB(0,0,0), RGB(0,0,0),
            RGB(0,0,0), RGB(0,0,0),
            RGB(0,0,0) 
        };

        for (spInt i = 0; i < 13; ++i)
        {
            boxes[i].body = spBodyNewDynamic();
            //boxes[i].shape = spPolygonNew();
            boxes[i].color = colors[i];
            boxes[i].border = RGB(0,0,0);
        }
    }
}

static void
Create()
{
    CreateWalls();
    CreateShapes();
    CreateConstraints();
}

static void 
Setup()
{
    demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    demo->world.gravity = vec(0,0);

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
}

static void 
Destroy()
{
}

spDemo* JointDemo = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });