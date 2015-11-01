
#ifndef SP_DEMO_H
#define SP_DEMO_H

#include "spDraw.h"
#include "spDemoPlatform.h"
#include <GLFW/glfw3.h>

typedef void (*initFunc)();
typedef void (*updateFunc)(spFloat dt);
typedef void (*destroyFunc)();
typedef void (*renderFunc)();
typedef void (*keyboardFunc)();

typedef GLFWwindow spWindow;
typedef spInt spDemoIndex;
typedef int spKey;

DEMO_API extern spFloat spLineScaleSmall;
DEMO_API extern spFloat spLineScaleBig;

typedef struct spMouse
{
    spConstraint* constraint;
    spShape* shape;
    spVector position;
} spMouse;

typedef struct spDemo
{
    spWorld world;
    spMouse mouse;
    spWindow* window;
    initFunc initialize;
    updateFunc update;
    destroyFunc destroy;
    keyboardFunc keyboard;
    spColor background;
    spFloat timestep;
    spFloat time;
    spFloat timePrev;
    spFloat timeAccum;
    spFloat ortho[16];
    spFloat view[16];
    spFrustum frustum;
    spViewport viewport;
    spBool paused;
} spDemo;

typedef struct spSingleBodyObject
{
    spBody* body;
    spShape* shape;
    spColor color;
    spColor border;
} spSingleBodyObject;

typedef struct spMultiBodyObject
{
    spBody* body;
    spShape** shapes;
    spColor* colors;
    spColor* borders;
    spInt count;
} spMultiBodyObject;

DEMO_API extern spDemo* Demo;

DEMO_API spDemo* spDemoNew(initFunc init, updateFunc update, destroyFunc destroy, spFrustum, spViewport view);

DEMO_API void spDemoSetCameraTranslation(spVector translation);

DEMO_API void spDemoFree(spDemo* demo);

DEMO_API void spDemoRun(spDemo* demo);

DEMO_API void spDemoInitRandomSeed();

DEMO_API spFloat spDemoRandomFloatRange(float min, float max);

DEMO_API spColor spDemoRandomColor();

DEMO_API spColor spDemoRandomPastelColor();

DEMO_API spBool spDemoKeyPressed(spKey key);

DEMO_API spBool spDemoKeyReleased(spKey key);

DEMO_API void spDemoDrawSingleBody(spSingleBodyObject* object);

DEMO_API void spDemoDrawMultiBody(spMultiBodyObject* object);

DEMO_API void spDemoDrawShape(spShape* shape, spColor color, spColor border);

DEMO_API void spDemoDrawConstraint(spConstraint* constraint);

DEMO_API void spDemoDrawCircle(spShape* shape, spColor color, spColor border);

DEMO_API void spDemoDrawPolygon(spShape* shape, spColor color, spColor border);

DEMO_API void spDemoDrawSegment(spShape* shape, spColor color, spColor border);

DEMO_API void spDemoDrawGearJoint(spConstraint* constraint, spColor color, spColor border);

DEMO_API void spDemoDrawAngularSpringJoint(spConstraint* constraint, spColor color, spColor border);

DEMO_API void spDemoDrawMotorJoint(spConstraint* constraint, spColor color, spColor border);

DEMO_API void spDemoDrawMouseJoint(spConstraint* constraint, spColor color, spColor cursor, spColor border);

DEMO_API void spDemoDrawRopeJoint(spConstraint* constraint, spColor circles, spColor rope, spColor border);

DEMO_API void spDemoDrawDistanceJoint(spConstraint* constraint, spColor color, spColor border);

DEMO_API void spDemoDrawPointJoint(spConstraint* constraint, spColor color, spColor border);

DEMO_API void spDemoDrawSpringJoint(spConstraint* constraint, spColor color, spColor border);

DEMO_API void spDemoDrawWheelJoint(spConstraint* constraint, spColor color, spColor border);

#endif