
#include "spDraw.h"
#include <GLFW/glfw3.h>

typedef void (*initFunc)();
typedef void (*updateFunc)(spFloat dt);
typedef void (*destroyFunc)();
typedef GLFWwindow spWindow;
typedef spInt spDemoIndex;

extern spFloat spLineScaleSmall;
extern spFloat spLineScaleBig;

struct spMouse
{
    spConstraint* constraint;
    spShape* shape;
    spVector position;
};

struct spDemo
{
    spWorld world;
    spMouse mouse;
    spWindow* window;
    initFunc initialize;
    updateFunc update;
    destroyFunc destroy;
    spColor background;
    spFloat timestep;
    spFloat time;
    spFloat timePrev;
    spFloat timeAccum;
    spFloat ortho[16];
    spFrustum frustum;
    spViewport viewport;
    spBool paused;
};

extern spDemo* demo;
extern spDemo* test;
extern spDemo* bridge;

spDemo* spDemoNew(initFunc init, updateFunc update, destroyFunc destroy, spFrustum, spViewport view);

void spDemoFree(spDemo** demo);

void spDemoDrawShape(spShape* shape, spColor color, spColor border);

void spDemoDrawConstraint(spConstraint* constraint);

void spRunDemo(spDemoIndex demo);