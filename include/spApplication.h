
#ifndef SP_APPLICATION_H
#define SP_APPLICATION_H

#include "spWorld.h"
#include "spMouseJoint.h"
#include <GLFW\glfw3.h>

/// viewport size
struct spViewport
{
    spInt width;
    spInt height;
};

/// ortho frustum parameters
struct spFrustum
{
    spFloat left;
    spFloat right;
    spFloat top;
    spFloat bottom;
    spFloat near;
    spFloat far;
};

/// function pointers
typedef void (*init_func)(struct spApplication*);
typedef void (*main_loop_func)(struct spApplication* app);
typedef void (*loop_func)(spApplication* app);

/// simple GLFW application
struct spApplication
{
    GLFWwindow* window;
    spWorld world;
    spFloat timestep;
    init_func init;
    loop_func loop; 
    main_loop_func main;
    spLazyPointer* data;
    spViewport viewport;
    spFrustum frustum;
    spMouseJoint* mouse;
    spShape* mouseShape;
    const spInt8* name;
};

/// allocate a new application on the heap
spApplication* spApplicationAlloc();

/// initialize an application
void spApplicationInit(
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
    spLazyPointer*    data);

/// allocate and init a new application on the heap
spApplication* spApplicationNew(
    const spInt8*     name,
    const spViewport& viewport,
    const spFrustum&  frustum,
    const spVector&   gravity,
    spInt             iterations,
    spFloat           timestep, 
    init_func         init, 
    loop_func         loop, 
    main_loop_func    main,
    spLazyPointer*    data);

/// free an applications memory
inline void spAplicationFree(spApplication** app)
{
    spFree(app);
}

/// 'faked' constructor for stack allocation
inline spViewport _spViewport(spInt width, spInt height)
{
    return { width, height };
}

/// 'faked' constructor for stack allocation
inline spFrustum _spFrustum(spFloat l, spFloat r, spFloat b, spFloat t, spFloat n, spFloat f)
{
    spFrustum frustum;
    frustum.left = l;
    frustum.right = r;
    frustum.bottom = b;
    frustum.top = t;
    frustum.near = n;
    frustum.far = f;
    return frustum;
}

/// convenience macros for stack allocators
#define spViewport(w, h) _spViewport(w, h)
#define spFrustum(l, r, b, t, n, f) _spFrustum(l, r, b, t, n, f)
#define spFrustumUniform(s) _spFrustum(-s, s, -s, s, -s, s)

///
void default_init(spApplication* app);

///
void default_main_loop(spApplication* app);

///
void default_loop(spApplication* app);

///
spInt run(spApplication* app);

#endif