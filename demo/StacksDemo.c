
#include "spDemoFactory.h"

#define MAXBOXES 8 

struct stackBox
{
    spShape* shape;
    spBody*  body;
    spColor  color;
};

static spBody* body[3];
static spShape* segment[3];
static stackBox boxes[MAXBOXES];
static spInt boxCount = 0;

static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
stackBoxCreate(spVector size, spFloat mass)
{
    spVector verts[4] = { {-size.x, -size.y}, { size.x, -size.y}, { size.x,  size.y}, {-size.x,  size.y}};

    spShape* shape = spPolygonNew(verts, 4, mass);
    spBody*  body = spBodyNewDynamic();

    spShapeSetNewMaterial(shape, spMaterialConstruct(0.0f, 1.0f));

    spBodyAddShape(body, shape);
    spWorldAddBody(&demo->world, body);

    boxes[boxCount].shape = shape;
    boxes[boxCount].body = body;

    boxCount += 1;
    spInt x = 0;
}

static void
stackBoxReset(spInt index, spVector position, spFloat angle)
{
    spBodyClearForces(boxes[index].body);
    spBodyClearVelocity(boxes[index].body);
    spBodySetTransform(boxes[index].body, position, angle);
}

const static spVector size = vec(60.0f, 50.0f);

static void
CreateBoxes()
{
    for (spInt i = 0; i < MAXBOXES; ++i)
    {
        spVector box = vec(size.x + ((MAXBOXES-1 * i) * 5.0f), size.y);
        stackBoxCreate(box, 300.0f);
    }
    spInt x = 0;
}

static void
ResetBoxes()
{
    spFloat y = -700.0f;
    for (spInt i = 0; i < MAXBOXES; ++i)
    {
        stackBoxReset(i, vec(0.0f, y), 0.0f);
        y += size.y*2.1f;
    }
}

static void
Keyboard()
{
}

static void
Setup()
{
    spSlop = 1.0f;
    spLineScaleSmall = 16.0f;
    spLineScaleBig = 20.0f;

    demo->background = RGBA255(176.f, 226.f, 255.f, 0.0f);
    demo->keyboard = Keyboard;

    segment[0] = spSegmentNew(vec(-1366.f, 768.f), vec(-1366.f,-768.f), 25.0f, 0.0f);
    segment[1] = spSegmentNew(vec(-1366.f,-768.f), vec( 1366.f,-768.f), 25.0f, 0.0f);
    segment[2] = spSegmentNew(vec( 1366.f,-768.f), vec( 1366.f, 768.f), 25.0f, 0.0f);

    body[0] = spBodyNewStatic();
    body[1] = spBodyNewStatic();
    body[2] = spBodyNewStatic();

    spBodyAddShape(body[0], segment[0]);
    spBodyAddShape(body[1], segment[1]);
    spBodyAddShape(body[2], segment[2]);

    spWorldAddBody(&demo->world, body[0]);
    spWorldAddBody(&demo->world, body[1]);
    spWorldAddBody(&demo->world, body[2]);

    CreateBoxes();
    ResetBoxes();
}

static void
Render()
{
    spDemoDrawShape(segment[0], WHITE(), WHITE());
    spDemoDrawShape(segment[1], WHITE(), WHITE());
    spDemoDrawShape(segment[2], WHITE(), WHITE());

    for (spInt i = 0; i < MAXBOXES; ++i)
    {
        spDemoDrawShape(boxes[i].shape, RED(), BLACK());
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

spDemo* stacks = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });