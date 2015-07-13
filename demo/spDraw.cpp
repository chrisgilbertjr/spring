
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
    GLFWwindow* window = glfwCreateWindow(demo.viewport.width, demo.viewport.height, demo.title, 0, 0);
    spAssert(window != NULL, "window creation failed\n");
    glfwMakeContextCurrent(window);
}

void spDrawInit()
{
    initGLFW3();
    initWindow();
}