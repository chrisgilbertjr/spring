
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
void create_box(spApplication* app, spBody** b, spPolygon** p, spFloat m, spVector pos, spFloat a, spFloat r, spFloat f, spFloat g, spVector size)
{
    *b = spCreateBody(&app->world, SP_BODY_DYNAMIC);
    spPolygonDef def;

    spVector verts[4];
    spVector s = size;
    spVectorSet(verts+0,-s.x,-s.y);
    spVectorSet(verts+1, s.x,-s.y);
    spVectorSet(verts+2, s.x, s.y);
    spVectorSet(verts+3,-s.x, s.y);

    def.mass = m;
    def.material.restitution = r;
    def.material.friction = f;
    def.vertex_count = 4;
    def.vertices = verts;
    *p = spCreatePolygon(*b, def);

    spBodySetTransform(*b, pos, a);
    spBody* body = *b;
    body->g_scale = g;
}

void
box_box_init(spApplication* app)
{
    static const spInt MAX_BODIES = 16;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];

    create_box(app, &bodies[0], &boxes[0], 9999999999.0f, spVector(0.0f, -4.0f), 0.0f, 0.01f, 0.8f, 0.0f, spVector(200.0f, 1.0f));
    for (spInt i = 1; i < MAX_BODIES; ++i)
    {
        spFloat f = (spFloat)i * 8.f;
        create_box(app, bodies+i, boxes+i, 1.0f, spVector(f * 0.1f - 16.0f, f), f*12.5f, 0.2f, 0.7f, 1.0f, spVector(3.0f, 4.0f));
        //create_box(app, bodies+i, boxes+i, 5.0f, spVector(f * 0.01f - 16.0f, f), 0.0f, 0.4f, 0.4f, 1.0f, spVector(3.0f, 4.0f));
    }
}

spApplication* box_box_collision()
{
    return spApplicationNew(
        "test application",
        spViewport(800, 800), spFrustumUniform(50.0f),
        spVector(0.0f, -9.8f),
        100, 1.0f / 60.0f,
        box_box_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------
void
distance_constraint_init(spApplication* app)
{
    static const spInt MAX_BODIES = 3;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spBody* a;
    spBody* b;

    create_box(app, &a, boxes+0, 99999999.0f, spVector(0.0f,  -10.0f), 5.0f, 0.9f, 1.0f, 0.0f, spVector(1000.0f, 1.0f));
    create_box(app, &a, boxes+1, 25.0f, spVector( 0.0f,  0.0f), 0.0f, 0.4f, 0.7f, 0.0f, spVector(4.0f, 4.0f));
    create_box(app, &b, boxes+2, 10.0f, spVector(2.0f, 25.0f), 0.0f, 0.4f, 0.7f, 1.0f, spVector(5.0f, 5.0f));
    a->g_scale = 1.0f;
    b->g_scale = 1.0f;
    spDistanceJoint* djoint = spDistanceConstraintNew(a, b, spVectorZero(), spVector(5.0f, 5.0f), 25.0f);
    spWorldAddDistanceJoint(&app->world, djoint);
}

spApplication* distance_constraint()
{
    return spApplicationNew(
        "distance constraint test app",
        spViewport(800, 800), spFrustumUniform(100.0f),
        spVector(0.0f, -9.8f),
        5, 1.0f / 60.0f,
        distance_constraint_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------

int main()
{
    return run(distance_constraint());
}