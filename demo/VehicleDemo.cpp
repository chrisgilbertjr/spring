
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
    TRUCK,
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

struct Truck
{
    Vehicle vehicle;

    spBody*  chasisBody;
    spBody*  chasisBodyB;
    spShape* chasisShapeA;
    spShape* chasisShapeB;
    spShape* chasisShapeC;

    spBody*  wheelBodyA;
    spBody*  wheelBodyB;
    spShape* wheelShapeA;
    spShape* wheelShapeB;

    spConstraint* chasisConstraintA;
    spConstraint* chasisConstraintB;
    spConstraint* wheelConstraintA;
    spConstraint* wheelConstraintB;
};

spVector translate = {0.0f, 0.0f};
Vehicle* currentVehicle;
Sedan sedan;
Truck truck;


/// TODO: mini car, monster truck, buggy

static Sedan
SedanNew()
{
    spFilter sedanFilter = spFilterConstruct(0<<0, spCollideAll, spCollideAll);

    spVector vertsA[4] = { vec(-400.f, -70.f), vec(400.f, -70.f), vec(350.f, 50.f), vec(-350.f, 50.f) };
    spVector vertsB[4] = { vec(-300.f,  0.f), vec(250.f,  0.f), vec(100.f,125.f), vec(-175.f,125.f) };

    spBody*  chasisBody   = spBodyNewDynamic();
    spBody*  wheelBodyA   = spBodyNewDynamic();
    spBody*  wheelBodyB   = spBodyNewDynamic();

    spBodySetTransform(chasisBody, vec(   0.f,   0.f), 0.f);
    spBodySetTransform(wheelBodyA, vec(-300.f,-250.f), 0.f);
    spBodySetTransform(wheelBodyB, vec( 300.f,-250.f), 0.f);

    spShape* chasisShapeA = spPolygonNew(vertsA, 4, 500.f);
    spShape* chasisShapeB = spPolygonNew(vertsB, 4, 400.f);
    spShape* wheelShapeA  = spCircleNew(spVectorZero(), 70.f, 150.f);
    spShape* wheelShapeB  = spCircleNew(spVectorZero(), 70.f, 150.f);

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

    spShapeSetFilter(chasisShapeA, sedanFilter);
    spShapeSetFilter(chasisShapeB, sedanFilter);
    spShapeSetFilter(wheelShapeA,  sedanFilter);
    spShapeSetFilter(wheelShapeB,  sedanFilter);

    spConstraint* wheelConstraintA = spWheelJointNew(chasisBody, wheelBodyA, vec(-275.f, -75.f), vec(0.f, 0.f), vec(0.f, 1.f), 3.5f, 0.8f);
    spConstraint* wheelConstraintB = spWheelJointNew(chasisBody, wheelBodyB, vec( 275.f, -75.f), vec(0.f, 0.f), vec(0.f, 1.f), 3.5f, 0.8f);

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

static Truck
TruckNew()
{
    spFilter truckFilter = spFilterConstruct(0<<1, spCollideAll, spCollideAll);

    spVector vertsA[4] = { vec(-500.f,-100.f), vec(500.f,-100.f), vec(500.f, 100.f), vec(-500.f, 100.f) };
    spVector vertsB[4] = { vec(-50.f, 50.f), vec(350.f, 50.f), vec(150.f, 200.f), vec(-50.f, 200.f) };
    spVector vertsC[4] = { vec(-475.f,-50.f), vec(475.f,-50.f), vec(475.f,  50.f), vec(-475.f,  50.f) };

    spBody*  chasisBody   = spBodyNewDynamic();
    spBody*  chasisBodyB  = spBodyNewDynamic();
    spBody*  wheelBodyA   = spBodyNewDynamic();
    spBody*  wheelBodyB   = spBodyNewDynamic();

    spBodySetTransform(wheelBodyA, vec(-300.f,-250.f), 0.f);
    spBodySetTransform(wheelBodyB, vec( 300.f,-250.f), 0.f);

    spShape* chasisShapeA = spPolygonNew(vertsA, 4, 500.f);
    spShape* chasisShapeB = spPolygonNew(vertsB, 4, 000.f);
    spShape* chasisShapeC = spPolygonNew(vertsC, 4, 400.f);
    spShape* wheelShapeA  = spCircleNew(spVectorZero(), 150.f, 150.f);
    spShape* wheelShapeB  = spCircleNew(spVectorZero(), 150.f, 150.f);

    spMaterial chasisMaterial = spMaterialConstruct(0.1f, 0.7f);
    spMaterial wheelMaterial = spMaterialConstruct(0.2f, 0.6f);

    spShapeSetNewMaterial(chasisShapeA, chasisMaterial);
    spShapeSetNewMaterial(chasisShapeB, chasisMaterial);
    spShapeSetNewMaterial(wheelShapeA, wheelMaterial);
    spShapeSetNewMaterial(wheelShapeB, wheelMaterial);

    spBodyAddShape(chasisBody, chasisShapeA);
    spBodyAddShape(chasisBody, chasisShapeB);
    spBodyAddShape(chasisBodyB, chasisShapeC);
    spBodyAddShape(wheelBodyA, wheelShapeA);
    spBodyAddShape(wheelBodyB, wheelShapeB);

    spShapeSetFilter(chasisShapeA, truckFilter);
    spShapeSetFilter(chasisShapeB, truckFilter);
    spShapeSetFilter(chasisShapeC, truckFilter);
    spShapeSetFilter(wheelShapeA,  truckFilter);
    spShapeSetFilter(wheelShapeB,  truckFilter);

    spConstraint* wheelConstraintA = spWheelJointNew(chasisBodyB, wheelBodyA, vec(-350.f, -300.f), vec(0.f, 0.f), vec(0.f, 1.f), 2.0f, 0.3f);
    spConstraint* wheelConstraintB = spWheelJointNew(chasisBodyB, wheelBodyB, vec( 350.f, -300.f), vec(0.f, 0.f), vec(0.f, 1.f), 2.0f, 0.3f);
    spWheelJointSetEnableMotor(wheelConstraintA, spTrue);
    spWheelJointSetEnableMotor(wheelConstraintB, spTrue);
    spWheelJointSetMaxMotorTorque(wheelConstraintA, 50000000.0f);
    spWheelJointSetMaxMotorTorque(wheelConstraintB, 50000000.0f);
    spWheelJointSetMotorSpeed(wheelConstraintA, 100000.0f);
    spWheelJointSetMotorSpeed(wheelConstraintB, 100000.0f);

    spConstraint* chasisConstraintA = spWheelJointNew(chasisBody, chasisBodyB, vec(-450.f, -100.f), vec(-450.f, 0.f), vec(0.f, 1.f), 2.0f, 0.5f);
    spConstraint* chasisConstraintB = spWheelJointNew(chasisBody, chasisBodyB, vec( 450.f, -100.f), vec( 450.f, 0.f), vec(0.f, 1.f), 2.0f, 0.5f);

    Truck truck;
    truck.vehicle.type = TRUCK;
    truck.chasisBody   = chasisBody;
    truck.chasisBodyB  = chasisBodyB;
    truck.chasisShapeA = chasisShapeA;
    truck.chasisShapeB = chasisShapeB;
    truck.chasisShapeC = chasisShapeC;
    truck.wheelBodyA   = wheelBodyA;
    truck.wheelBodyB   = wheelBodyB;
    truck.wheelShapeA  = wheelShapeA;
    truck.wheelShapeB  = wheelShapeB;
    truck.wheelConstraintA = wheelConstraintA;
    truck.wheelConstraintB = wheelConstraintB;
    truck.chasisConstraintA = chasisConstraintA;
    truck.chasisConstraintB = chasisConstraintB;

    return truck;
}

static void 
AddVehicle()
{
    if (currentVehicle->type == TRUCK)
    {
        spWorldAddBody(&demo->world, truck.chasisBody);
        spWorldAddBody(&demo->world, truck.chasisBodyB);
        spWorldAddBody(&demo->world, truck.wheelBodyA);
        spWorldAddBody(&demo->world, truck.wheelBodyB);
        spWorldAddConstraint(&demo->world, truck.wheelConstraintA);
        spWorldAddConstraint(&demo->world, truck.wheelConstraintB);
        spWorldAddConstraint(&demo->world, truck.chasisConstraintA);
        spWorldAddConstraint(&demo->world, truck.chasisConstraintB);
    }
    else if (currentVehicle->type == SEDAN)
    {
        spWorldAddBody(&demo->world, sedan.chasisBody);
        spWorldAddBody(&demo->world, sedan.wheelBodyA);
        spWorldAddBody(&demo->world, sedan.wheelBodyB);
        spWorldAddConstraint(&demo->world, sedan.wheelConstraintA);
        spWorldAddConstraint(&demo->world, sedan.wheelConstraintB);
    }
}

static void
RemoveVehicle()
{
    if (currentVehicle->type == TRUCK)
    {
        spWorldRemoveBody(&demo->world, truck.chasisBody);
        spWorldRemoveBody(&demo->world, truck.chasisBodyB);
        spWorldRemoveBody(&demo->world, truck.wheelBodyA);
        spWorldRemoveBody(&demo->world, truck.wheelBodyB);
        spWorldRemoveConstraint(&demo->world, truck.wheelConstraintA);
        spWorldRemoveConstraint(&demo->world, truck.wheelConstraintB);
        spWorldRemoveConstraint(&demo->world, truck.chasisConstraintA);
        spWorldRemoveConstraint(&demo->world, truck.chasisConstraintB);
    }
    else if (currentVehicle->type == SEDAN)
    {
        spWorldRemoveBody(&demo->world, sedan.chasisBody);
        spWorldRemoveBody(&demo->world, sedan.wheelBodyA);
        spWorldRemoveBody(&demo->world, sedan.wheelBodyB);
        spWorldRemoveConstraint(&demo->world, sedan.wheelConstraintA);
        spWorldRemoveConstraint(&demo->world, sedan.wheelConstraintB);
    }
}

static void
ResetVehicle()
{
    if (currentVehicle->type == TRUCK)
    {
        spBodySetTransform(truck.chasisBody, spVectorZero(), 0.0f);
        spBodySetTransform(truck.chasisBodyB, spVectorZero(), 0.0f);
        spBodySetTransform(truck.wheelBodyA, vec(-300.f,-250.f), 0.f);
        spBodySetTransform(truck.wheelBodyB, vec( 300.f,-250.f), 0.f);
    }
}


static void
DrawVehicle()
{
    if (currentVehicle->type == TRUCK)
    {
        spDemoDrawShape(truck.chasisShapeC, RGB(0,0,0), RGB(0,0,0));
        spDemoDrawShape(truck.chasisShapeB, RGB(0.8f, 0.5f, 0.0f), RGB(0,0,0));
        spDemoDrawShape(truck.chasisShapeA, RGB(0.8f, 0.5f, 0.0f), RGB(0,0,0));
        spDemoDrawShape(truck.wheelShapeA,  RGB(0.5f, 0.0f, 0.8f), RGB(0,0,0));
        spDemoDrawShape(truck.wheelShapeB,  RGB(0.5f, 0.0f, 0.8f), RGB(0,0,0));
        spDemoDrawWheelJoint(truck.wheelConstraintA, RGBA(0,1,0,0.5f), BLACK());
        spDemoDrawWheelJoint(truck.wheelConstraintB, RGBA(0,1,0,0.5f), BLACK());
        spDemoDrawWheelJoint(truck.chasisConstraintA, RGBA(0,1,0,0.2f), BLACK());
        spDemoDrawWheelJoint(truck.chasisConstraintB, RGBA(0,1,0,0.2f), BLACK());
    }
}

static void
Setup()
{
    spSlop = 2.0f;
    spLineScaleSmall = 12.0f;
    spLineScaleBig = 20.0f;

    demo->background = RGBA255(176.f, 226.f, 255.f, 0.0f);

    srand((unsigned int)time(NULL));

    sedan = SedanNew();
    truck = TruckNew();

    currentVehicle = &truck.vehicle;
    AddVehicle();

    spBody* body = spBodyNewStatic();
    spShape* segment = spSegmentNew(vec(-2000.f, -600.f), vec(2000.f, -600.f), 5.0f, 0.f);

    spBodyAddShape(body, segment);
    spWorldAddBody(&demo->world, body);
}

static void
Render()
{
    //spDemoDrawShape(sedan.chasisShapeB, RGB(0.5f, 0.0f, 0.5f), RGB(0,0,0));
    //spDemoDrawShape(sedan.chasisShapeA, RGB(0.5f, 0.0f, 0.5f), RGB(0,0,0));
    //spDemoDrawShape(sedan.wheelShapeA,  RGB(0.0f, 0.5f, 0.5f), RGB(0,0,0));
    //spDemoDrawShape(sedan.wheelShapeB,  RGB(0.0f, 0.5f, 0.5f), RGB(0,0,0));
    //spDemoDrawWheelJoint(sedan.wheelConstraintA, RGB(0,1,0), BLACK());
    //spDemoDrawWheelJoint(sedan.wheelConstraintB, RGB(0,1,0), BLACK());

    DrawVehicle();

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