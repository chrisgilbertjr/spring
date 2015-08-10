
#include "spDemoFactory.h"

static spConstraint* hinges[2];
static spShape* pendulumShapes[2];
static spBody*  pendulumBodies[2];
static spShape* staticShape;
static spBody*  staticBody;

static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static void
Create()
{
    spVector size = vec(20.f, 200.f);
    spVector verts[4] = { vec(-size.x,-size.y), vec(size.x, -size.y), vec(size.x, size.y), vec(-size.x, size.y) };

    spGroup pendulum = 0;
    spFilter filter = spFilterConstruct(pendulum, spCollideAll, spCollideAll);

    staticShape = spCircleNew(vec(0.f, 0.f), 5.f, 0.f);
    staticBody  = spBodyNewStatic();

    pendulumShapes[0] = spPolygonNew(verts, 4, 1000.f);
    pendulumShapes[1] = spPolygonNew(verts, 4, 1000.f);
    pendulumBodies[0] = spBodyNewDynamic();
    pendulumBodies[1] = spBodyNewDynamic();

    hinges[0] = spPointJointNew(staticBody, pendulumBodies[0], vec(0,0), vec(0, -size.y+10.f));
    hinges[1] = spPointJointNew(pendulumBodies[0], pendulumBodies[1], vec(0,size.y-10.f), vec(0,-size.y+10.f));

    spShapeSetFilter(staticShape, filter);
    spShapeSetFilter(pendulumShapes[0], filter);
    spShapeSetFilter(pendulumShapes[1], filter);

    spBodyAddShape(pendulumBodies[0], pendulumShapes[0]);
    spBodyAddShape(pendulumBodies[1], pendulumShapes[1]);
    spBodyAddShape(staticBody, staticShape);

    spBodySetLinearVelocityDamping(pendulumBodies[0], 0.f);
    spBodySetLinearVelocityDamping(pendulumBodies[1], 0.f);
    spBodySetAngularVelocityDamping(pendulumBodies[0], 0.f);
    spBodySetAngularVelocityDamping(pendulumBodies[1], 0.f);

    spWorldAddBody(&demo->world, pendulumBodies[0]);
    spWorldAddBody(&demo->world, pendulumBodies[1]);
    spWorldAddBody(&demo->world, staticBody);

    spWorldAddConstraint(&demo->world, hinges[0]);
    spWorldAddConstraint(&demo->world, hinges[1]);
}

static void
Reset()
{
    spBodySetTransform(staticBody, vec(0.f, 0.f), 0.f);
    spBodySetTransform(pendulumBodies[0], vec(0.f, 200.f), 0.f);
    spBodySetTransform(pendulumBodies[1], vec(0.f, 600.f), 0.f);

    spBodyClearForces(staticBody);
    spBodyClearForces(pendulumBodies[0]);
    spBodyClearForces(pendulumBodies[1]);

    spBodyClearVelocity(staticBody);
    spBodyClearVelocity(pendulumBodies[0]);
    spBodyClearVelocity(pendulumBodies[1]);
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

    Create();
    Reset();
}

static void
Render()
{
    spDemoDrawShape(pendulumShapes[0], RGB(1,0.5,0), RGB(0,0,0));
    spDemoDrawShape(pendulumShapes[1], RGB(1,0.5,0), RGB(0,0,0));
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

spDemo* pendulum = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });