
#include "demo\spDemo.h"

typedef spSingleBodyObject box;
typedef spSingleBodyObject ball;
typedef spSingleBodyObject wall;

#define BOXES 9 
#define BALLS 8

static ball balls[BALLS];
static box  boxes[BOXES];
static wall walls[8];
static wall pin;

static spConstraint* gear;
static spConstraint* motor;
static spConstraint* angSpring;
static spConstraint* distance;
static spConstraint* rope;
static spConstraint* spring;
static spConstraint* wheels[3];
static spConstraint* point;

/// wall positions
spVector wallpos[16] = { 
{ -33.3f,  -33.3f}, { -33.3f,  100.0f},
{  33.3f, -100.0f}, {  33.3f,  100.0f},
{-100.0f,   33.3f}, { 100.0f,   33.3f},
{-100.0f,  -33.3f}, { 100.0f,  -33.3f},
{-100.0f, -100.0f}, { 100.0f, -100.0f},
{-100.0f,  100.0f}, { 100.0f,  100.0f},
{-100.0f, -100.0f}, {-100.0f,  100.0f},
{ 100.0f, -100.0f}, { 100.0f,  100.0f},};

/// for easy poly creation
static spVector poly[5] = { 0 };

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
    /// gear joint
    spDemoSingleBodyTransform(balls+0, vec(-76.5f, 66.6f), 0.0f);
    spDemoSingleBodyTransform(balls+1, vec(-46.5f, 66.6f), 0.0f);

    /// motor joint
    spDemoSingleBodyTransform(boxes+0, vec(-15.f, 66.6f), 0.0f);
    spDemoSingleBodyTransform(boxes+1, vec( 15.f, 66.6f), 0.0f);

    /// ang spring joint
    spDemoSingleBodyTransform(boxes+2, vec( 52.5f, 66.6f), 0.0f);
    spDemoSingleBodyTransform(boxes+3, vec( 82.5f, 66.6f), 0.0f);

    /// distance joint
    spDemoSingleBodyTransform(balls+2, vec(-82.5f, 0.0f), 0.0f);
    spDemoSingleBodyTransform(balls+3, vec(-52.5f, 0.0f), 0.0f);

    /// rope joint
    spDemoSingleBodyTransform(balls+4, vec(-15.0f, 0.0f), 0.0f);
    spDemoSingleBodyTransform(balls+5, vec( 15.0f, 0.0f), 0.0f);

    /// spring joint
    spDemoSingleBodyTransform(boxes+4, vec( 52.5f, 0.0f), 0.0f);
    spDemoSingleBodyTransform(boxes+5, vec( 82.5f, 0.0f), 0.0f);

    /// wheel joint
    spDemoSingleBodyTransform(boxes+6, vec(-66.6f, -75.f), 0.0f);
    spDemoSingleBodyTransform(balls+6, vec(-80.6f, -90.f), 0.0f);
    spDemoSingleBodyTransform(balls+7, vec(-52.6f, -90.f), 0.0f);

    /// point joint
    spDemoSingleBodyTransform(boxes+7, vec( 52.5f,-66.6f), 0.0f);
    spDemoSingleBodyTransform(boxes+8, vec( 82.5f,-66.6f), 0.0f);
}

static void
Create()
{
    /// create pin for rotating objects
    pin.body = spBodyNewStatic();
    pin.shape = spCircleNew(vec(1000,1000), 1, 100);

    /// create walls
    for (spInt i = 0; i < 8; ++i)
    {
        walls[i].shape = spSegmentNew(wallpos[i * 2], wallpos[i * 2 + 1], 1.0f, 0.0f);
        walls[i].body = spBodyNewStatic();
        walls[i].color = RGB(0, 0, 0);
        walls[i].border = RGB(0, 0, 0);

        spDemoAttachSingleBody(walls+i);
    };

    /// gear joint
    balls[0].body = spBodyNewDynamic();
    balls[1].body = spBodyNewDynamic();
    balls[0].shape = spCircleNew(vec(0,0), 20.0f, 30.0f);
    balls[1].shape = spCircleNew(vec(0,0), 10.0f, 15.0f);
    balls[0].color = RGB255(129,255,51);
    balls[1].color = RGB255(255,51,102);
    balls[1].border = RGB(0,0,0);
    balls[0].border = RGB(0,0,0);

    spDemoAttachSingleBody(balls+0);
    spDemoAttachSingleBody(balls+1);

    gear = spGearJointNew(balls[0].body, balls[1].body, 2.0f, 0.0f);
    spWorldAddConstraint(&Demo->world, spPointJointNew(pin.body, balls[0].body, vec(-76.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, spPointJointNew(pin.body, balls[1].body, vec(-46.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, gear);

    /// motor joint
    boxes[0].body = spBodyNewDynamic();
    boxes[1].body = spBodyNewDynamic();
    boxes[0].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[1].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[0].color = RGB255(255,163,51);
    boxes[1].color = RGB255(255,51,235);
    boxes[0].border = RGB(0,0,0);
    boxes[1].border = RGB(0,0,0);

    motor = spMotorJointNew(boxes[0].body, boxes[1].body, 5.f);
    spWorldAddConstraint(&Demo->world, spPointJointNew(pin.body, boxes[0].body, vec(-15.0f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, spPointJointNew(pin.body, boxes[1].body, vec( 15.0f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, motor);

    spDemoAttachSingleBody(boxes+0);
    spDemoAttachSingleBody(boxes+1);
    
    /// angular spring joint
    boxes[2].body = spBodyNewDynamic();
    boxes[3].body = spBodyNewDynamic();
    boxes[2].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[3].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[2].color = RGB255(116,255,51);
    boxes[3].color = RGB255(255,119,51);
    boxes[2].border = RGB(0,0,0);
    boxes[3].border = RGB(0,0,0);

    spDemoAttachSingleBody(boxes+2);
    spDemoAttachSingleBody(boxes+3);

    angSpring = spAngularSpringJointNew(boxes[2].body, boxes[3].body, spFalse, 1.0f, 0.2f, 0.0f);
    spWorldAddConstraint(&Demo->world, spPointJointNew(pin.body, boxes[2].body, vec( 52.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, spPointJointNew(pin.body, boxes[3].body, vec( 82.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, angSpring);

    /// distance joint
    balls[2].body = spBodyNewDynamic();
    balls[3].body = spBodyNewDynamic();
    balls[2].shape = spCircleNew(vec(0,0), 7.0f, 50.0f);
    balls[3].shape = spCircleNew(vec(0,0), 7.0f, 50.0f);
    balls[2].color = RGB255(62,48,255);
    balls[3].color = RGB255(241,255,48);
    balls[2].border = RGB(0,0,0);
    balls[3].border = RGB(0,0,0);

    distance = spDistanceJointNew(balls[2].body, balls[3].body, vec(0,-6), vec(0,6), 30.f);
    spWorldAddConstraint(&Demo->world, distance);

    spDemoAttachSingleBody(balls+2);
    spDemoAttachSingleBody(balls+3);

    /// rope joint
    balls[4].body = spBodyNewDynamic();
    balls[5].body = spBodyNewDynamic();
    balls[4].shape = spCircleNew(vec(0,0), 7.0f, 50.0f);
    balls[5].shape = spCircleNew(vec(0,0), 7.0f, 50.0f);
    balls[4].color = RGB255(51,51,255);
    balls[5].color = RGB255(255,102,51);
    balls[4].border = RGB(0,0,0);
    balls[5].border = RGB(0,0,0);

    spDemoAttachSingleBody(balls+4);
    spDemoAttachSingleBody(balls+5);

    rope = spRopeJointNew(balls[4].body, balls[5].body, vec(0,0), vec(0,0), 25.0f);
    spWorldAddConstraint(&Demo->world, rope);

    /// spring joint
    boxes[4].body = spBodyNewDynamic();
    boxes[5].body = spBodyNewDynamic();
    boxes[4].shape = spPolygonNew(polyNew(6,6), 4, 50.0f);
    boxes[5].shape = spPolygonNew(polyNew(6,6), 4, 50.0f);
    boxes[4].color = RGB255(255,51,143);
    boxes[5].color = RGB255(163,51,255);
    boxes[4].border = RGB(0,0,0);
    boxes[5].border = RGB(0,0,0);

    spDemoAttachSingleBody(boxes+4);
    spDemoAttachSingleBody(boxes+5);

    spring = spSpringJointNew(boxes[4].body, boxes[5].body, vec(5,5), vec(-5,5), 1.0f, 0.1f, 10.0f);
    spWorldAddConstraint(&Demo->world, spring);
        
    /// wheel joint
    boxes[6].body = spBodyNewDynamic();
    balls[6].body = spBodyNewDynamic();
    balls[7].body = spBodyNewDynamic();
    boxes[6].shape = spPolygonNew(polyNew(20, 6), 4, 50.0f);
    balls[6].shape = spCircleNew(vec(0,0), 6, 20.f);
    balls[7].shape = spCircleNew(vec(0,0), 6, 20.f);
    boxes[6].color = RGB255(248,51,255);
    balls[6].color = RGB255(255,150,51);
    balls[7].color = RGB255(255,150,51);
    boxes[6].border = RGB(0,0,0);
    balls[6].border = RGB(0,0,0);
    balls[7].border = RGB(0,0,0);
    spShapeSetMaterial(balls[6].shape, 0.0f, 1.0f);
    spShapeSetMaterial(balls[7].shape, 0.0f, 1.0f);

    spGroup carGroup = 1;
    spFilter car = spFilterConstruct(carGroup, spCollideAll, spCollideAll);

    spShapeSetFilter(boxes[6].shape, car);
    spShapeSetFilter(balls[6].shape, car);
    spShapeSetFilter(balls[7].shape, car);

    wheels[0] = spWheelJointNew(boxes[6].body, balls[6].body, vec(-14,-15), vec(0,0), vec(0,1), 2.0f, 0.5f);
    wheels[1] = spWheelJointNew(boxes[6].body, balls[7].body, vec( 14,-15), vec(0,0), vec(0,1), 2.0f, 0.5f);

    spWorldAddConstraint(&Demo->world, wheels[0]);
    spWorldAddConstraint(&Demo->world, wheels[1]);

    spDemoAttachSingleBody(balls+6);
    spDemoAttachSingleBody(balls+7);
    spDemoAttachSingleBody(boxes+6);

    /// point joint
    boxes[7].body = spBodyNewDynamic();
    boxes[8].body = spBodyNewDynamic();
    boxes[7].shape = spPolygonNew(polyNew(6,6), 4, 50.0f);
    boxes[8].shape = spPolygonNew(polyNew(6,6), 4, 50.0f);
    boxes[7].color = RGB255(255,55,48);
    boxes[8].color = RGB255(138,255,48);
    boxes[7].border = RGB(0,0,0);
    boxes[8].border = RGB(0,0,0);

    spDemoAttachSingleBody(boxes+7);
    spDemoAttachSingleBody(boxes+8);

    point = spPointJointNew(boxes[7].body, boxes[8].body, vec(15,0), vec(-15,0));
    spWorldAddConstraint(&Demo->world, point);

}

static void 
Keyboard()
{
    if (spDemoKeyPressed('R')) Reset();
    if (spDemoKeyPressed('G'))
    {
        if (spvEqual(Demo->world.gravity, vec(0, 0)))
        {
            Demo->world.gravity = spDemoGravity();
        }
        else
        {
            Demo->world.gravity = vec(0,0);
        }
    }
}

static void 
Setup()
{
    spDemoSetKeyCallback(Keyboard);

    spLineScaleSmall = 1.5f;
    spLineScaleBig = 2.0f;
    spSlop = 0.5f;

    Create();
    Reset();
}

static void
Render()
{
    for (spInt i = 0; i < 8; ++i)
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

    spDemoDrawConstraint(gear, RGB255(51,255,153), RGB(0,0,0));
    spDemoDrawConstraint(motor, RGB255(51,255,92), RGB(0,0,0));
    spDemoDrawConstraint(angSpring, RGB255(190,51,255), RGB(0,0,0));
    spDemoDrawConstraint(distance, RGB255(255,55,48), RGB(0,0,0));
    spDemoDrawConstraint(rope, RGB255(255,204,51), RGB(0,0,0));
    spDemoDrawConstraint(spring, RGB255(245,255,51), RGB(0,0,0));
    spDemoDrawConstraint(wheels[0], RGBA255(58,255,51,128), RGBA(0,0,0,.5));
    spDemoDrawConstraint(wheels[1], RGBA255(58,255,51,128), RGBA(0,0,0,.5));
    spDemoDrawConstraint(point, RGB255(48,277,255), RGB(0,0,0));
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
    spShapeFree(&pin.shape);
    spBodyFree(&pin.body);
}

static void 
Run()
{
    spDemoRun(spDemoNew(Setup, Update, Render, Destroy, spFrustumView(100, 100), spViewportNew(800, 800)));
}

/// extern function pointer
Joints = Run;