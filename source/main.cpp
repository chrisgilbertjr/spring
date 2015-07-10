
#include <spring.h> 
#include "spApplication.h"

#define PRESSED(key) glfwGetKey(app->window, key) == GLFW_PRESS
#define RELEASED(key) glfwGetKey(app->window, key) == GLFW_RELEASE

void init_test(spApplication* app)
{
    spBody* body;
    spShape* box;

    body = spBodyNewStatic();
    box = spSegmentNew(spVectorConstruct(-10.0f, 0.0f), spVectorConstruct(10.0f, 0.0f), 5.0f, 10.0f);
    box->body = body;
    spBodySetTransform(body, spVectorConstruct(150.0f, 50.0f), 0.0f);
    spWorldAddBody(&app->world, body);
    spBodyAddShape(body, box);

    body = spBodyNewStatic();
    box = spSegmentNew(spVectorConstruct(-20.0f, 0.0f), spVectorConstruct(20.0f, 0.0f), 10.0f, 10.0f);
    box->body = body;
    spBodySetTransform(body, spVectorConstruct(0.0f, 50.0f), 0.0f);
    spWorldAddBody(&app->world, body);
    spBodyAddShape(body, box);
}

spApplication* test()
{
    return spApplicationNew(
        "test app",
        spViewport(800, 800), spFrustumUniform(250), 
        spVectorConstruct(0.f, -98.f), 5, 1.f/60.f, 
        init_test, default_loop, default_main_loop, NULL);
}

int main()
{
    return run(test());
}