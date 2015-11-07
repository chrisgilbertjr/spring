
#include "demo\spDemo.h"

typedef spSingleBodyObject box;
typedef spSingleBodyObject ball;
typedef spSingleBodyObject ball;
typedef spSingleBodyObject wall;

#define BOXES 4
#define BALLS 6

static ball balls[BALLS];
static box  boxes[BOXES];
static wall walls[8];

static spConstraint* gear;
static spConstraint* motor;
static spConstraint* angSpring;
static spConstraint* distance;
static spConstraint* rope;

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
spVector poly[5] = { 0 };

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

static 
spVector* triNew(spFloat min, spFloat max)
{
    spFloat mid = spAbs(max - min);
    poly[0] = vec(-mid, -mid);
    poly[1] = vec( mid, -mid);
    poly[2] = vec(0.0f,  mid);
    return poly;
}

static void 
Keyboard()
{
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
}

static void
Create()
{
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
    balls[0].color = RGB255(177, 59, 255);
    balls[1].color = RGB255(59, 216, 255);
    balls[1].border = RGB(0,0,0);
    balls[0].border = RGB(0,0,0);
    spDemoAttachSingleBody(balls+0);
    spDemoAttachSingleBody(balls+1);
    gear = spGearJointNew(balls[0].body, balls[1].body, 2.0f, 0.0f);
    spWorldAddConstraint(&Demo->world, spPointJointNew(walls[0].body, balls[0].body, vec(-76.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, spPointJointNew(walls[1].body, balls[1].body, vec(-46.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, gear);

    /// motor joint
    boxes[0].body = spBodyNewDynamic();
    boxes[1].body = spBodyNewDynamic();
    boxes[0].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[1].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[0].color = RGB(1,0,0);
    boxes[1].color = RGB(0,1,0);
    boxes[0].border = RGB(0,0,0);
    boxes[1].border = RGB(0,0,0);
    motor = spMotorJointNew(boxes[0].body, boxes[1].body, 5.f);
    spDemoAttachSingleBody(boxes+0);
    spDemoAttachSingleBody(boxes+1);
    spWorldAddConstraint(&Demo->world, spPointJointNew(walls[0].body, boxes[0].body, vec(-15.0f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, spPointJointNew(walls[1].body, boxes[1].body, vec( 15.0f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, motor);
    
    /// angular spring joint
    boxes[2].body = spBodyNewDynamic();
    boxes[3].body = spBodyNewDynamic();
    boxes[2].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[3].shape = spPolygonNew(polyNew(14, 3), 4, 50.0f);
    boxes[2].color = RGB(1,0,0);
    boxes[3].color = RGB(0,1,0);
    boxes[2].border = RGB(0,0,0);
    boxes[3].border = RGB(0,0,0);
    angSpring = spAngularSpringJointNew(boxes[2].body, boxes[3].body, spFalse, 1.0f, 0.2f, 0.0f);
    spDemoAttachSingleBody(boxes+2);
    spDemoAttachSingleBody(boxes+3);
    spWorldAddConstraint(&Demo->world, spPointJointNew(walls[0].body, boxes[2].body, vec( 52.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, spPointJointNew(walls[1].body, boxes[3].body, vec( 82.5f, 66.6f), vec(0,0)));
    spWorldAddConstraint(&Demo->world, angSpring);

    /// distance joint
    balls[2].body = spBodyNewDynamic();
    balls[3].body = spBodyNewDynamic();
    balls[2].shape = spCircleNew(vec(0,0), 10.0f, 50.0f);
    balls[3].shape = spCircleNew(vec(0,0), 10.0f, 50.0f);
    balls[2].color = RGB(0,1,1);
    balls[3].color = RGB(1,0,0);
    balls[2].border = RGB(0,0,0);
    balls[3].border = RGB(0,0,0);
    spDemoAttachSingleBody(balls+2);
    spDemoAttachSingleBody(balls+3);
    distance = spDistanceJointNew(balls[2].body, balls[3].body, vec(0,-5), vec(0,5), 30.f);
    spWorldAddConstraint(&Demo->world, distance);

    /// rope joint
    balls[4].body = spBodyNewDynamic();
    balls[5].body = spBodyNewDynamic();
    balls[4].shape = spCircleNew(vec(0,0), 7.0f, 50.0f);
    balls[5].shape = spCircleNew(vec(0,0), 7.0f, 50.0f);
    balls[4].color = RGB(1,1,0);
    balls[5].color = RGB(1,0,1);
    balls[4].border = RGB(0,0,0);
    balls[5].border = RGB(0,0,0);
    spDemoAttachSingleBody(balls+4);
    spDemoAttachSingleBody(balls+5);
    rope = spRopeJointNew(balls[4].body, balls[5].body, vec(0,0), vec(0,0), 25.0f);
    spWorldAddConstraint(&Demo->world, rope);
}

static void 
Setup()
{
    Demo->background = RGBA255(176.f, 226.f, 255.f, 0.f);
    //Demo->world.gravity = vec(0,0);

    spLineScaleSmall = 1.5f;
    spLineScaleBig = 3.0f;
    spSlop = 0.2f;

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


    spDemoDrawConstraint(gear, RGB255(255, 167, 59), RGB(0,0,0));
    spDemoDrawConstraint(motor, RGB(0,1,1), RGB(0,0,0));
    spDemoDrawConstraint(angSpring, RGB(1,0,1), RGB(0,0,0));
    spDemoDrawConstraint(distance, RGB(1,1,0), RGB(0,0,0));
    spDemoDrawConstraint(rope, RGB(1,0,0), RGB(0,0,0));
    spDemoDrawMouse();
}

static void
Update(spFloat dt)
{
    spWorldStep(&Demo->world, dt);
    Render();
}

static void 
Destroy()
{
}

int main(void)
{
    spDemoRun(spDemoNew(Setup, Update, Render, Destroy, spFrustumView(100, 100), spViewportNew(800, 800)));
    return 0;
}