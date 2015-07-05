
#include "spApplication.h"

spApplication* 
spApplicationAlloc()
{
    return (spApplication*)spMalloc(sizeof(spApplication));
}

void
spApplicationInit(
    spApplication*    app,
    const spInt8*     name,
    const spViewport& viewport,
    const spFrustum&  frustum,
    const spVector&   gravity,
    spInt             iterations,
    spFloat           timestep, 
    init_func         init, 
    loop_func         loop, 
    main_loop_func    main,
    spLazyPointer*    data)
{
    app->name = name;
    app->viewport = viewport;
    app->frustum = frustum;
    app->world = spWorldConstruct(gravity);
    app->world.iterations = iterations;
    app->timestep = timestep;
    app->init = init;
    app->loop = loop;
    app->main = main;
    app->data = data;
    app->mouse = spMouseJointNew(NULL, 1.5f, 0.5f, spVectorZero(), spVectorZero());
    app->mouseShape = NULL;
}

spApplication* 
spApplicationNew(
    const spInt8*     name,
    const spViewport& viewport,
    const spFrustum&  frustum,
    const spVector&   gravity,
    spInt             iterations,
    spFloat           timestep, 
    init_func         init, 
    loop_func         loop, 
    main_loop_func    main,
    spLazyPointer*    data)
{
    spApplication* app = spApplicationAlloc();
    spApplicationInit(
        app, 
        name, 
        viewport, 
        frustum, 
        gravity, 
        iterations, 
        timestep, 
        init, 
        loop, 
        main, 
        data);
    return app;
}

void 
default_init(spApplication* app)
{
}

spBool
init_glfw(spApplication* app)
{
    if (!glfwInit())
    {
        return spFalse;
    }

    app->window = glfwCreateWindow(app->viewport.width, app->viewport.height, app->name, 0, 0);
    
    if (!app->window)
    {
        glfwTerminate();
        return spFalse;
    }

    glfwMakeContextCurrent(app->window);

    return spTrue;
}

#define PRESSED(key) glfwGetKey(app->window, key) == GLFW_PRESS
#define RELEASED(key) glfwGetKey(app->window, key) == GLFW_RELEASE

void 
default_main_loop(spApplication* app)
{
    {
        spFloat l = app->frustum.left;
        spFloat r = app->frustum.right;
        spFloat b = app->frustum.bottom;
        spFloat t = app->frustum.top; spFloat n = app->frustum.near; spFloat f = app->frustum.far;

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(l, r, b, t, n, f);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    glfwSetTime(0.0);
    while(!glfwWindowShouldClose(app->window))
    {
        spFloat dt = (spFloat)glfwGetTime() / 1000.f;
        while (dt < app->timestep)
        {
            dt = (spFloat)glfwGetTime();
        }
        glClear(GL_COLOR_BUFFER_BIT);

        if (PRESSED(' '))
        {
            app->loop(app);
        }
        else
        {
            app->loop(app);
        }

        glfwSwapBuffers(app->window);

        glfwPollEvents();
        glfwSetTime(0.0);
    }

    glfwDestroyWindow(app->window);
    glfwTerminate();
}

void 
default_loop(spApplication* app)
{
    double x, y;
    glfwGetCursorPos(app->window, &x, &y);
    x =  x / app->viewport.width;
    y = -y / app->viewport.height + 1;
    spFloat w = spAbs(app->frustum.right - app->frustum.left);
    spFloat h = spAbs(app->frustum.top - app->frustum.bottom);
    x *= w; x -= w*0.5f;
    y *= h; y -= h*0.5f;
    spVector pos = spVector((spFloat)x, (spFloat)y);

    if (glfwGetMouseButton(app->window, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS)
    {
        if (app->mouseShape == NULL)
        {
            spShape* shape = spWorldTestPoint(&app->world, pos);

            if (shape != NULL && shape->body->type == SP_BODY_DYNAMIC)
            {
                spMouseJointStart(app->mouse, shape->body, pos);
                spWorldAddMouseJoint(&app->world, app->mouse);
                app->mouse->constraint.body_a = shape->body;
                app->mouseShape = shape;
            }
        }
        else
        {
            spMouseJointUpdate(app->mouse, pos);
        }
    }
    else
    {
        if (app->mouseShape != NULL)
        {
            spMouseJointEnd(app->mouse);
            spWorldRemoveMouseJoint(&app->world, app->mouse);
        }
        app->mouseShape = NULL;
    }

    spWorldStep(&app->world, app->timestep);
}

spInt 
run(spApplication* app)
{
    spAssert(app != NULL, "application is NULL!");
    if (init_glfw(app) == spFalse) return -1;
    app->init(app);
    app->main(app); 
    spAplicationFree(&app);
    return 0;
}