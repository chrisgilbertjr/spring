
#include "spDraw.h"
#include "spDemo.h"

static void
initGLFW3()
{
    spAssert(glfwInit(), "GLFW failed to initialize\n");
}

static void
initWindow()
{
    GLFWwindow* window = glfwCreateWindow((int)demo.viewport.width, (int)demo.viewport.height, NULL, 0, 0);
    spAssert(window != NULL, "window creation failed\n");
    glfwMakeContextCurrent(window);
}

void spDrawInit()
{
    initGLFW3();
    initWindow();
}

void spDrawPolygon()
{
}

void spDrawSegment()
{
}

void spDrawCircle(spVector center, spFloat angle, spFloat radius)
{
}

void spDrawPoint()
{
}