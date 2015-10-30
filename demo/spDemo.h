
#include "spDraw.h"
#include <GLFW/glfw3.h>

typedef void (*initFunc)();
typedef void (*updateFunc)(spFloat dt);
typedef void (*destroyFunc)();
typedef void (*renderFunc)();
typedef void (*keyboardFunc)();

typedef GLFWwindow spWindow;
typedef spInt spDemoIndex;
typedef int spKey;

extern spFloat spLineScaleSmall;
extern spFloat spLineScaleBig;

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

extern spDemo* Demo;

spDemo* spDemoNew(initFunc init, updateFunc update, destroyFunc destroy, spFrustum, spViewport view);

void spDemoSetCameraTranslation(spVector translation);

void spDemoFree(spDemo* demo);

void spDemoRun(spDemo* demo);

void spDemoInitRandomSeed();

spFloat spDemoRandomFloatRange(float min, float max);

spColor spDemoRandomColor();

spColor spDemoRandomPastelColor();

spBool spDemoKeyPressed(spKey key);

spBool spDemoKeyReleased(spKey key);

void spDemoDrawSingleBody(spSingleBodyObject* object);

void spDemoDrawMultiBody(spMultiBodyObject* object);

void spDemoDrawShape(spShape* shape, spColor color, spColor border);

void spDemoDrawConstraint(spConstraint* constraint);

void spDemoDrawCircle(spShape* shape, spColor color, spColor border);

void spDemoDrawPolygon(spShape* shape, spColor color, spColor border);

void spDemoDrawSegment(spShape* shape, spColor color, spColor border);

void spDemoDrawGearJoint(spConstraint* constraint, spColor color, spColor border);

void spDemoDrawAngularSpringJoint(spConstraint* constraint, spColor color, spColor border);

void spDemoDrawMotorJoint(spConstraint* constraint, spColor color, spColor border);

void spDemoDrawMouseJoint(spConstraint* constraint, spColor color, spColor cursor, spColor border);

void spDemoDrawRopeJoint(spConstraint* constraint, spColor circles, spColor rope, spColor border);

void spDemoDrawDistanceJoint(spConstraint* constraint, spColor color, spColor border);

void spDemoDrawPointJoint(spConstraint* constraint, spColor color, spColor border);

void spDemoDrawSpringJoint(spConstraint* constraint, spColor color, spColor border);

void spDemoDrawWheelJoint(spConstraint* constraint, spColor color, spColor border);