
#include "spDraw.h"

struct spTimer
{
    spFloat dt;
    spFloat ms;
};

struct spMouse
{
    spFloat x;
    spFloat y;
};

struct spViewport
{
    spFloat width;
    spFloat height;
};

struct spFrustum
{
    spFloat left;
    spFloat right;
    spFloat top;
    spFloat bottom;
    spFloat near;
    spFloat far;
};

struct spDemo
{
    spRenderContext context;
    spViewport viewport;
    spFrustum frustum;
    spTimer timer;
    spMouse mouse;
};

typedef void (*initFunc)();
typedef void (*updateFunc)();
typedef void (*destroyFunc)();

extern spDemo demo;