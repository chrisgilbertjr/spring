
#include "spDraw.h"
#include <GLFW/glfw3.h>

typedef void (*initFunc)();
typedef void (*updateFunc)(spFloat dt);
typedef void (*destroyFunc)();
typedef GLFWwindow spWindow;
typedef spInt spDemoIndex;

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
    spBool paused;
};

extern spDemo* demo;
extern spDemo* test;

spDemo* spDemoNew(initFunc init, updateFunc update, destroyFunc destroy);

void spDemoFree(spDemo** demo);

void spDemoDrawShape(spShape* shape, spColor color, spColor border);

void spDemoDrawConstraint(spConstraint* constraint, spColor color, spColor border);

void spRunDemo(spDemoIndex demo);