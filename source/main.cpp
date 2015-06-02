
#include <spring.h> 
#include "spApplication.h"

#define PRESSED(key) glfwGetKey(app->window, key) == GLFW_PRESS
#define RELEASED(key) glfwGetKey(app->window, key) == GLFW_RELEASE

//---------------------------------------------------------------------------------------------------------------------
struct app_data
{
    spPolygon* pa;
    spPolygon* pb;
    spBody* ba;
    spBody* bb;
};

void
init_test(spApplication* app)
{
    app_data* ad = (app_data*) spMalloc(sizeof(app_data));
    ad->ba = spCreateBody(&app->world, SP_BODY_DYNAMIC);
    ad->bb = spCreateBody(&app->world, SP_BODY_DYNAMIC);
    spPolygonDef def;

    spVector verts[4];
    spFloat size = 2.0f;
    spVectorSet(verts+0,-size,-size);
    spVectorSet(verts+1, size,-size);
    spVectorSet(verts+2, size, size);
    spVectorSet(verts+3,-size, size);

    def.mass = 1.0f;
    def.material.restitution = 0.5f;
    def.material.friction = 0.5f;
    def.vertex_count = 4;
    def.vertices = verts;
    ad->pa = spCreatePolygon(ad->ba, def);
    ad->pb = spCreatePolygon(ad->bb, def);
    ad->pa->base_class.bound.half_width = spVector(100.0f, 100.0f);
    ad->pb->base_class.bound.half_width = spVector(100.0f, 100.0f);

    spBodySetPosition(ad->ba, spVector(0.0f, 0.0f));
    spBodySetPosition(ad->bb, spVector(0.0f, 0.0f));
    ad->ba->g_scale = 0.0f;
    ad->bb->g_scale = 0.0f;
    app->data = (spLazyPointer*)ad;
}

void 
loop_test(spApplication* app)
{
    app_data* ad = (app_data*)app->data;

    spPolygon* pa = ad->pa;
    spPolygon* pb = ad->pb;
    spBody* ba = ad->ba;
    spBody* bb = ad->bb;

    spFloat bax = ba->xf.p.x;
    spFloat bay = ba->xf.p.y;
    spFloat bbx = bb->xf.p.x;
    spFloat bby = bb->xf.p.y;
    spFloat aa = spRotationGetAngleDeg(ba->xf.q);
    spFloat ab = spRotationGetAngleDeg(bb->xf.q);

    spFloat speed = 0.1f;

    if (PRESSED('D')) bax += speed;
    if (PRESSED('A')) bax -= speed;
    if (PRESSED('W')) bay += speed;
    if (PRESSED('S')) bay -= speed;
    if (PRESSED('Q')) aa -= 1.0f;
    if (PRESSED('E')) aa += 1.0f;

    if (PRESSED('L')) bbx += speed;
    if (PRESSED('J')) bbx -= speed;
    if (PRESSED('I')) bby += speed;
    if (PRESSED('K')) bby -= speed;
    if (PRESSED('U')) ab -= 1.0f;
    if (PRESSED('O')) ab += 1.0f;

    spBodySetTransform(ad->ba, spVector(bax, bay), aa);
    spBodySetTransform(ad->bb, spVector(bbx, bby), ab);

    default_loop(app);
}

spApplication* test_application()
{
    return spApplicationNew(
        "test application",
        spViewport(800, 800), spFrustumUniform(10.0f),
        spVector(0.0f, -1.0f),
        20, 1.0f / 60.0f,
        init_test, loop_test, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
spApplication* fall_application()
{
    return spApplicationNew(
        "test application",
        spViewport(800, 800), spFrustumUniform(10.0f),
        spVector(0.0f, -1.0f),
        20, 1.0f / 1000.0f,
        default_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------

int main()
{
    return run(test_application());
}