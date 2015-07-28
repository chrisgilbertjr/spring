
#include <time.h>
#include "spDemoFactory.h"

/// vector helper function for convenience
static spVector
vec(spFloat x, spFloat y)
{
    return spVectorConstruct(x, y);
}

struct vehicleA
{
    spDemoShape* sBody;
    spDemoShape* sWheelA;
    spDemoShape* sWheelB;
    spConstraint* cWheelA;
    spConstraint* cWheelB;
};

static vehicleA
CreateVehicleA()
{
    spInt body   = spCreateDynamicBox(vec(0,0), 0, vec(400,100), 500.f, RGB(0.5, 0, 0.5), RGB(0,0,0));
    spInt wheelA = spCreateDynamicCircle(vec(-300,-250), 100, 200, RGB(0, 0.5, 0.5), RGB(0,0,0));
    spInt wheelB = spCreateDynamicCircle(vec( 300,-250), 100, 200, RGB(0, 0.5, 0.5), RGB(0,0,0));
    spConstraint* cWheelA = spAddConstraint(spWheelJointNew(Body(body), Body(wheelA), vec(-300, -200), vec(0,0), vec(0,1), 1.5f, 0.5f));
    spConstraint* cWheelB = spAddConstraint(spWheelJointNew(Body(body), Body(wheelB), vec( 300, -200), vec(0,0), vec(0,1), 1.5f, 0.5f));

    spGroup carGroup = 0<<1;
    spFilter carFilter = spFilterConstruct(carGroup, spCollideAll, spCollideAll);

    spShapeSetFilter(Shape(body),   carFilter);
    spShapeSetFilter(Shape(wheelA), carFilter);
    spShapeSetFilter(Shape(wheelB), carFilter);

    return { DemoShape(body), DemoShape(wheelA), DemoShape(wheelB), cWheelA, cWheelB };
}

/// camera translate
spVector translate = {0.0f, 0.0f};
vehicleA VehicleA;

static void
SetCamera()
{
    translate = VehicleA.sBody->body->p;
}

static spFloat
RandomFloatRange(float min, float max) 
{
    float random = ((float)rand()) / (float)RAND_MAX;
    float range = max - min;
    return min + random * range;
}

static void
Setup()
{
    spSlop = 2.0f;
    spLineScaleSmall = 12.0f;
    spLineScaleBig = 20.0f;

    demo->background = RGBA255(176.f, 226.f, 255.f, 0.0f);

    srand((unsigned int)time(NULL));

    demo->world.gravity = vec(0.0f, 0.0f);

    VehicleA = CreateVehicleA();
}

static void 
Update(spFloat dt)
{
    spWorldStep(&demo->world, dt);
    spDemoSetCameraTranslation(translate);
    spDrawFactoryObjects();
}

static void 
Destroy()
{
}

spDemo* vehicle = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });