
#include <time.h>
#include "spDemoFactory.h"

/// vector helper function for convenience
static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

static spFloat
RandomFloatRange(float min, float max) 
{
    float random = ((float)rand()) / (float)RAND_MAX;
    float range = max - min;
    return min + random * range;
}

enum vehicleType
{
    SEDAN,
};

struct Vehicle
{
    vehicleType type;
};

struct Sedan
{
    Vehicle  vehicle;

    spBody*  chasisBody;
    spShape* chasisShapeA;
    spShape* chasisShapeB;

    spBody*  wheelBodyA;
    spBody*  wheelBodyB;
    spShape* wheelShapeA;
    spShape* wheelShapeB;

    spConstraint* wheelConstraintA;
    spConstraint* wheelConstraintB;
};

/// TODO: mini car, monster truck, buggy


static Sedan
SedanNew()
{
    spFilter sedanFilter = spFilterConstruct(0<<0, spCollideAll, spCollideAll);

    spVector vertsA[4] = { vec(-400.f, -70.f), vec(400.f, -70.f), vec(400.f, 70.f), vec(-400.f, 70.f) };
    spVector vertsB[4] = { vec(-300.f,  50.f), vec(250.f,  50.f), vec(100.f,175.f), vec(-175.f,175.f) };

    spBody*  chasisBody   = spBodyNewDynamic();
    spBody*  wheelBodyA   = spBodyNewDynamic();
    spBody*  wheelBodyB   = spBodyNewDynamic();

    spBodySetTransform(chasisBody, vec(   0.f,   0.f), 0.f);
    spBodySetTransform(wheelBodyA, vec(-300.f,-250.f), 0.f);
    spBodySetTransform(wheelBodyB, vec( 300.f,-250.f), 0.f);

    spShape* chasisShapeA = spPolygonNew(vertsA, 4, 500.f);
    spShape* chasisShapeB = spPolygonNew(vertsB, 4, 100.f);
    spShape* wheelShapeA  = spCircleNew(spVectorZero(), 80.f, 150.f);
    spShape* wheelShapeB  = spCircleNew(spVectorZero(), 80.f, 150.f);

    spMaterial chasisMaterial = spMaterialConstruct(0.1f, 0.7f);
    spMaterial wheelMaterial = spMaterialConstruct(0.2f, 0.6f);

    spShapeSetNewMaterial(chasisShapeA, chasisMaterial);
    spShapeSetNewMaterial(chasisShapeB, chasisMaterial);
    spShapeSetNewMaterial(wheelShapeA, wheelMaterial);
    spShapeSetNewMaterial(wheelShapeB, wheelMaterial);

    spBodyAddShape(chasisBody, chasisShapeA);
    spBodyAddShape(chasisBody, chasisShapeB);
    spBodyAddShape(wheelBodyA, wheelShapeA);
    spBodyAddShape(wheelBodyB, wheelShapeB);
    spBodySetCenterOfMass(chasisBody, spVectorZero());

    spShapeSetFilter(chasisShapeA, sedanFilter);
    spShapeSetFilter(chasisShapeB, sedanFilter);
    spShapeSetFilter(wheelShapeA,  sedanFilter);
    spShapeSetFilter(wheelShapeB,  sedanFilter);

    spConstraint* wheelConstraintA = spWheelJointNew(chasisBody, wheelBodyA, vec(-275.f, -75.f), vec(0.f, 0.f), vec(0.f, 1.f), 3.5f, 0.8f);
    spConstraint* wheelConstraintB = spWheelJointNew(chasisBody, wheelBodyB, vec( 275.f, -75.f), vec(0.f, 0.f), vec(0.f, 1.f), 3.5f, 0.8f);

    /// add this to an add function
    spWorldAddBody(&demo->world, chasisBody);
    spWorldAddBody(&demo->world, wheelBodyA);
    spWorldAddBody(&demo->world, wheelBodyB);
    spWorldAddConstraint(&demo->world, wheelConstraintA);
    spWorldAddConstraint(&demo->world, wheelConstraintB);

    Sedan sedan;
    sedan.vehicle.type = SEDAN;
    sedan.chasisBody = chasisBody;
    sedan.chasisShapeA = chasisShapeA;
    sedan.chasisShapeB = chasisShapeB;
    sedan.wheelBodyA = wheelBodyA;
    sedan.wheelBodyB = wheelBodyB;
    sedan.wheelShapeA = wheelShapeA;
    sedan.wheelShapeB = wheelShapeB;
    sedan.wheelConstraintA = wheelConstraintA;
    sedan.wheelConstraintB = wheelConstraintB;

    return sedan;
}

/// camera translate
spVector translate = {0.0f, 0.0f};
Sedan sedan;

static void
Setup()
{
    spSlop = 2.0f;
    spLineScaleSmall = 12.0f;
    spLineScaleBig = 20.0f;

    demo->background = RGBA255(176.f, 226.f, 255.f, 0.0f);

    srand((unsigned int)time(NULL));

    demo->world.gravity = vec(0.0f, 0.0f);

    sedan = SedanNew();
}

static void
Render()
{
    spDemoDrawShape(sedan.chasisShapeB, RGB(0.5f, 0.0f, 0.5f), RGB(0,0,0));
    spDemoDrawShape(sedan.chasisShapeA, RGB(0.5f, 0.0f, 0.5f), RGB(0,0,0));
    spDemoDrawShape(sedan.wheelShapeA,  RGB(0.0f, 0.5f, 0.5f), RGB(0,0,0));
    spDemoDrawShape(sedan.wheelShapeB,  RGB(0.0f, 0.5f, 0.5f), RGB(0,0,0));
    spDemoDrawWheelJoint(sedan.wheelConstraintA, RGB(0,1,0), BLACK());
    spDemoDrawWheelJoint(sedan.wheelConstraintB, RGB(0,1,0), BLACK());
    if (demo->mouse.constraint != NULL && demo->mouse.shape != NULL) 
    {
        spDemoDrawMouseJoint(demo->mouse.constraint, RGB(0,1,0), WHITE(), BLACK());
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

spDemo* vehicle = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });