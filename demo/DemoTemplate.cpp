
#include "spDemo.h"
#include "spPolygon.h"
#include "spSegment.h"
#include "spCircle.h"

typedef spSingleBodyObject spBox;
typedef spSingleBodyObject spBall;
typedef spSingleBodyObject spWall;

static spConstraint constraints[9];
static spBox  boxes[9];
static spBall balls[8];
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
Create()
{
}

static void
Setup()
{
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

spDemo* DEMO_NAME = spDemoNew(Setup, Update, Destroy, spFrustumView(1366, 768), { 1366, 768 });