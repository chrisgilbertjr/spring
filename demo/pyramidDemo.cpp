

#include "spDemo.h"
#include "spPolygon.h"
#include "spSegment.h"

typedef spSingleBodyObject pyramidBox;
typedef spSingleBodyObject wall;

static const spInt boxCount = 21;
static pyramidBox boxes[boxCount];
static wall walls[4];

static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
CreateWalls()
{
    walls[0].shape = spSegmentNew(vec(-1366.f, 768.f), vec(-1366.f,-768.f), 25.0f, 0.0f);
    walls[1].shape = spSegmentNew(vec(-1366.f,-768.f), vec( 1366.f,-768.f), 25.0f, 0.0f);
    walls[2].shape = spSegmentNew(vec( 1366.f,-768.f), vec( 1366.f, 768.f), 25.0f, 0.0f);
    walls[3].shape = spSegmentNew(vec(-1366.f, 768.f), vec( 1366.f, 768.f), 25.0f, 0.0f);

    walls[0].body = spBodyNewStatic();
    walls[1].body = spBodyNewStatic();
    walls[2].body = spBodyNewStatic();
    walls[3].body = spBodyNewStatic();

    walls[0].color = RGB(1,1,1);  walls[0].border = RGB(1,1,1);
    walls[1].color = RGB(1,1,1);  walls[1].border = RGB(1,1,1);
    walls[2].color = RGB(1,1,1);  walls[2].border = RGB(1,1,1);
    walls[3].color = RGB(1,1,1);  walls[3].border = RGB(1,1,1);

    spBodyAddShape(walls[0].body, walls[0].shape);
    spBodyAddShape(walls[1].body, walls[1].shape);
    spBodyAddShape(walls[2].body, walls[2].shape);
    spBodyAddShape(walls[3].body, walls[3].shape);

    spWorldAddBody(&demo->world, walls[0].body);
    spWorldAddBody(&demo->world, walls[1].body);
    spWorldAddBody(&demo->world, walls[2].body);
    spWorldAddBody(&demo->world, walls[3].body);
}

static void
CreateBox(spInt index)
{
    spVector size = vec(100.f, 100.f);
    spVector verts[4] = { {-size.x, -size.y}, { size.x, -size.y}, { size.x,  size.y}, {-size.x,  size.y}};

    spShape* shape  = spPolygonNew(verts, 4, spsqrt(size.x * size.y));
    spBody*  body   = spBodyNewDynamic();
    spColor  color  = spDemoRandomPastelColor();
    spColor  border = RGB(0,0,0);

    spShapeSetMaterial(shape, 0.4f, 0.7f);

    spBodyAddShape(body, shape);
    spWorldAddBody(&demo->world, body);

    boxes[index].body   = body;
    boxes[index].shape  = shape;
    boxes[index].color  = color;
    boxes[index].border = border;
}

static void
Create()
{
    CreateWalls();

    for (spInt i = 0; i < boxCount; ++i)
    {
        CreateBox(i);
    }
}

static void
Reset()
{
    spFloat start  = -600.f;
    spFloat yOffset = 200.f;
    spFloat xOffset = yOffset * 0.5f;

    for (spInt i = 0; i < boxCount; ++i)
    {
        spBodyClearForces(boxes[i].body);
        spBodyClearVelocity(boxes[i].body);
    }

    spBodySetTransform(boxes[0].body, vec(0.f, start + yOffset * 5.f), 0.f);

    spBodySetTransform(boxes[1].body, vec(-xOffset, start + yOffset * 4.f), 0.f);
    spBodySetTransform(boxes[2].body, vec( xOffset, start + yOffset * 4.f), 0.f);

    spBodySetTransform(boxes[3].body, vec(-xOffset * 2.0f, start + yOffset * 3.f), 0.f);
    spBodySetTransform(boxes[4].body, vec(            0.f, start + yOffset * 3.f), 0.f);
    spBodySetTransform(boxes[5].body, vec( xOffset * 2.0f, start + yOffset * 3.f), 0.f);

    spBodySetTransform(boxes[6].body, vec(-xOffset * 3.0f, start + yOffset * 2.f), 0.f);
    spBodySetTransform(boxes[7].body, vec(-xOffset * 1.0f, start + yOffset * 2.f), 0.f);
    spBodySetTransform(boxes[8].body, vec( xOffset * 1.0f, start + yOffset * 2.f), 0.f);
    spBodySetTransform(boxes[9].body, vec( xOffset * 3.0f, start + yOffset * 2.f), 0.f);

    spBodySetTransform(boxes[10].body, vec(-xOffset * 4.0f, start + yOffset * 1.f), 0.f);
    spBodySetTransform(boxes[11].body, vec(-xOffset * 2.0f, start + yOffset * 1.f), 0.f);
    spBodySetTransform(boxes[12].body, vec(           0.0f, start + yOffset * 1.f), 0.f);
    spBodySetTransform(boxes[13].body, vec( xOffset * 2.0f, start + yOffset * 1.f), 0.f);
    spBodySetTransform(boxes[14].body, vec( xOffset * 4.0f, start + yOffset * 1.f), 0.f);

    spBodySetTransform(boxes[15].body, vec(-xOffset * 5.0f, start + yOffset * 0.f), 0.f);
    spBodySetTransform(boxes[16].body, vec(-xOffset * 3.0f, start + yOffset * 0.f), 0.f);
    spBodySetTransform(boxes[17].body, vec(-xOffset * 1.0f, start + yOffset * 0.f), 0.f);
    spBodySetTransform(boxes[18].body, vec( xOffset * 1.0f, start + yOffset * 0.f), 0.f);
    spBodySetTransform(boxes[19].body, vec( xOffset * 3.0f, start + yOffset * 0.f), 0.f);
    spBodySetTransform(boxes[20].body, vec( xOffset * 5.0f, start + yOffset * 0.f), 0.f);
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
    for (spInt i = 0; i < 4; ++i)
    {
        spDemoDrawShape(walls[i].shape, walls[i].color, walls[i].border);
    }

    for (spInt i = 0; i < boxCount; ++i)
    {
        spDemoDrawShape(boxes[i].shape, boxes[i].color, boxes[i].border);
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

spDemo* pyramid = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });