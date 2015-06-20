
#include <spring.h> 
#include "spMotorJoint.h"
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
    body->v_damp = .0f;
}

void create_circle(spApplication* app, spBody** b, spCircle** c, spVector pos, spFloat ang, spFloat rad, spFloat mass, spFloat r, spFloat f, spFloat g)
{
    *b = spCreateBody(&app->world, SP_BODY_DYNAMIC);
    spCircleDef def;

    def.center = spVectorZero();
    def.mass = mass;
    def.material.restitution = r;
    def.material.friction = f;
    def.radius = rad;
    *c = spCreateCircle(*b, def);
    spBodySetTransform(*b, pos, ang);
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
    }
}

spApplication* box_box_collision()
{
    return spApplicationNew(
        "test application",
        spViewport(800, 800), spFrustumUniform(50.0f),
        spVector(0.0f, -19.6f),
        100, 1.0f / 60.0f,
        box_box_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------
void
distance_constraint_init(spApplication* app)
{
    static const spInt MAX_BODIES = 3;
    spPolygon* boxes[MAX_BODIES];
    spBody* a;
    spBody* b;

    create_box(app, &a, boxes+0, 99999999.0f, spVector(0.0f,  -10.0f), 25.0f, 0.1f, 1.0f, 0.0f, spVector(1000.0f, 1.0f));
    create_box(app, &a, boxes+1, 100.0f, spVector( 0.0f,  0.0f), 0.0f, 0.4f, 0.7f, 0.0f, spVector(4.0f, 4.0f));
    create_box(app, &b, boxes+2, 10.0f, spVector(2.0f, 25.0f), 0.0f, 0.4f, 0.2f, 1.0f, spVector(5.0f, 5.0f));
    a->g_scale = 1.0f;
    b->g_scale = 1.0f;
    spDistanceJoint* djoint = spDistanceConstraintNew(a, b, spVectorZero(), spVector(0.0f, 0.0f), 25.0f);
    spWorldAddDistanceJoint(&app->world, djoint);
}

spApplication* distance_constraint()
{
    return spApplicationNew(
        "distance constraint test app",
        spViewport(800, 800), spFrustumUniform(100.0f),
        spVector(0.0f, -90.8f),
        20, 1.0f / 60.0f,
        distance_constraint_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------
void
rope_constraint_init(spApplication* app)
{
    static const spInt MAX_BODIES = 12;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spRopeJoint* rope[MAX_BODIES];

    spFloat x = 0.0f;
    spFloat y = 85.0f;
    spFloat h = 2.0f;
    spFloat w = 1.0f;

    create_box(app, bodies+0, boxes+0, 999999.0f, spVector(0.0f, y), 90.0f, 0.4f, 0.5f, 0.0f, spVector(1.0f, 1.0f));

    for (spInt i = 1; i < MAX_BODIES; ++i)
    {
        x = (spFloat)i*h*3.0f;
        spFloat mass = i * 1.0f + 25.0f;
        create_box(app, bodies+i, boxes+i, mass, spVector(x, y), 90.0f, 0.4f, 0.5f, 1.0f, spVector(w, h));
    }

    rope[0] = spRopeJointNew(bodies[0], bodies[1], spVector(0.0f, 0.0f), spVector(0.0f, h), h);
    spWorldAddRopeJoint(&app->world, rope[0]);

    for (spInt i = 1; i < MAX_BODIES-1; ++i)
    {
        rope[i] = spRopeJointNew(bodies[i], bodies[i+1], spVector(0.0f, -h), spVector(0.0f, h), h*.4f);
        spWorldAddRopeJoint(&app->world, rope[i]);
    }
    //spBody* b2[2];
    //spPolygon* bx2[2];

    //create_box(app, b2+0, bx2+0, 99999999.0f, spVector(10.0f, 35.0f), 90.0f, 0.4f, 0.5f, 0.0f, spVector(8.0f, 8.0f));
    //create_box(app, b2+1, bx2+1, 99999999.0f, spVector(-20.0f, -20.0f), 90.0f, 0.4f, 0.5f, 0.0f, spVector(8.0f, 8.0f));
}

spApplication* rope_constraint()
{
    return spApplicationNew(
        "rope constraint test app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        rope_constraint_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------

void
motor_constraint_init(spApplication* app)
{
    static const spInt MAX_BODIES = 16;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spMotorJoint* motor[MAX_BODIES];

    spFloat x = 0.0f;
    spFloat y = 85.0f;
    spFloat h = 2.0f;
    spFloat w = 1.0f;
    spFloat d = 3.0f;

    create_box(app, bodies+0, boxes+0, 999999.0f, spVector( 40.0f, 0.0f), 90.0f, 0.4f, 0.5f, 0.0f, spVector(10.0f, 10.0f));
    create_box(app, bodies+1, boxes+1, 999999.0f, spVector(-40.0f, 0.0f), 90.0f, 0.4f, 0.5f, 0.0f, spVector(10.0f, 10.0f));

    create_box(app, bodies+2, boxes+2, 20.0f, spVector(0.0, 5.0f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+3, boxes+3, 20.0f, spVector(0.0,-5.0f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+4, boxes+4, 20.0f, spVector(1.0, 10.0f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+5, boxes+5, 20.0f, spVector(1.0,-10.0f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+6, boxes+6, 20.0f, spVector(2.0, 10.0f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+7, boxes+7, 20.0f, spVector(2.0,-10.0f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    spRopeJoint* rope0 = spRopeJointNew(bodies[0], bodies[2], spVector( 10.0f,  10.0f), spVector(0.0f, 5.0f), d);
    spRopeJoint* rope1 = spRopeJointNew(bodies[0], bodies[3], spVector(-10.0f, -10.0f), spVector(0.0f, 5.0f), d);
    spRopeJoint* rope2 = spRopeJointNew(bodies[2], bodies[4], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f),   d);
    spRopeJoint* rope3 = spRopeJointNew(bodies[3], bodies[5], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f),   d);
    spRopeJoint* rope4 = spRopeJointNew(bodies[4], bodies[6], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f),   d);
    spRopeJoint* rope5 = spRopeJointNew(bodies[5], bodies[7], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f),   d);

    create_box(app, bodies+8, boxes+8, 20.0f, spVector(0.0, 5.1f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+9, boxes+9, 20.0f, spVector(0.0,-5.1f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+10, boxes+10, 20.0f, spVector(1.0, 10.1f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+11, boxes+11, 20.0f, spVector(1.0,-10.1f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+12, boxes+12, 20.0f, spVector(2.0, 10.1f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    create_box(app, bodies+13, boxes+13, 20.0f, spVector(2.0,-10.1f), 90.0f, 0.4f, 0.5f, 1.0f, spVector(1.0f, 5.0f));
    spRopeJoint* arope0 = spRopeJointNew(bodies[1], bodies[8], spVector( 10.0f,  10.0f), spVector(0.0f, 5.0f), d);
    spRopeJoint* arope1 = spRopeJointNew(bodies[1], bodies[9], spVector(-10.0f, -10.0f), spVector(0.0f, 5.0f), d);
    spRopeJoint* arope2 = spRopeJointNew(bodies[8], bodies[10], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f),  d);
    spRopeJoint* arope3 = spRopeJointNew(bodies[9], bodies[11], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f),  d);
    spRopeJoint* arope4 = spRopeJointNew(bodies[10], bodies[12], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f), d);
    spRopeJoint* arope5 = spRopeJointNew(bodies[11], bodies[13], spVector(-1.0f, -5.0f), spVector(0.0f, 5.0f), d);

    spMotorJoint* joint = spMotorJointNew(bodies[0], bodies[1], 5.0f);

    spWorldAddRopeJoint(&app->world, rope0);
    spWorldAddRopeJoint(&app->world, rope1);
    spWorldAddRopeJoint(&app->world, rope2);
    spWorldAddRopeJoint(&app->world, rope3);
    spWorldAddRopeJoint(&app->world, rope4);
    spWorldAddRopeJoint(&app->world, rope5);
    spWorldAddRopeJoint(&app->world, arope0);
    spWorldAddRopeJoint(&app->world, arope1);
    spWorldAddRopeJoint(&app->world, arope2);
    spWorldAddRopeJoint(&app->world, arope3);
    spWorldAddRopeJoint(&app->world, arope4);
    spWorldAddRopeJoint(&app->world, arope5);
    spWorldAddMotorJoint(&app->world, joint);
}

spApplication* motor_constraint()
{
    return spApplicationNew(
        "motor constraint test app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        motor_constraint_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------
void
spring_init(spApplication* app)
{
    static const spInt MAX_BODIES = 2;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spSpringJoint* spring[MAX_BODIES];

    create_box(app, bodies+0, boxes+0, 100000.0f, spVector(0.0f, 0.0f), 0.0f, 0.4f, 0.5f, 0.0f, spVector(10.0f, 10.0f));
    create_box(app, bodies+1, boxes+1, 10.0f, spVector(100.0f, -0.0f), 0.0f, 0.4f, 0.5f, 1.0f, spVector(10.0f, 10.0f));
    bodies[0]->v_damp = 1.0f;
    bodies[1]->w_damp = 0.1f;
    bodies[1]->v_damp = 0.1f;

    spring[0] = spSpringJointNew(bodies[0], bodies[1], spVector(0.0f, -10.0f), spVector(-10.0f, 10.0f), 0.5f, 0.1f, 10.0f);
    spWorldAddSpringJoint(&app->world, spring[0]);
}

spApplication* spring()
{
    return spApplicationNew(
        "motor constraint test app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        spring_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------

void
angspring_init(spApplication* app)
{
    static const spInt MAX_BODIES = 2;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spAngularSpringJoint* spring[MAX_BODIES];

    create_box(app, bodies+0, boxes+0, 1.0f, spVector(-20.0f, 0.0f), 0.0f, 0.4f, 0.5f, 0.0f, spVector(10.0f, 10.0f));
    create_box(app, bodies+1, boxes+1, 1.0f, spVector( 20.0f, 0.0f), 0.0f, 0.4f, 0.5f, 0.0f, spVector(10.0f, 10.0f));

    spring[0] = spAngularSpringJointNew(bodies[0], bodies[1], spTrue, 1.0f, 0.0f, 135.0f * SP_DEG_TO_RAD);
    spWorldAddAngularSpringJoint(&app->world, spring[0]);
}

spApplication* angspring()
{
    return spApplicationNew(
        "motor constraint test app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        angspring_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------

void
wheel_init(spApplication* app)
{
    static const spInt MAX_BODIES = 4;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spBody* cbodies[MAX_BODIES];
    spCircle* circle[MAX_BODIES];
    spWheelJoint* wheel[MAX_BODIES];


    create_box(app, bodies+0, boxes+0, 100.0f, spVector(  0.0f, 50.0f), 0.0f, 0.4f, 0.5f, 1.0f, spVector(60.0f, 20.0f));
    create_box(app, bodies+3, boxes+3, 999999.0f, spVector(  0.0f, -20.0f), 0.0f, 0.4f, 0.5f, 0.0f, spVector(100.0f, 1.0f));
    create_circle(app, cbodies+0, circle+0, spVector(-50.0f, 0.0f), 0.0f, 10.0f, 10.0f, 0.9f, 0.1f, 1.0f);
    create_circle(app, cbodies+1, circle+1, spVector( 50.0f, 0.0f), 0.0f, 10.0f, 10.0f, 0.9f, 0.1f, 1.0f);

    wheel[0] = spWheelJointNew(bodies[0], cbodies[0], spVector(-50.0f, -50.0f), spVector(0.0f, 0.0f), spVector(0.0f, 1.0f), 2.0f, 0.2f);
    wheel[1] = spWheelJointNew(bodies[0], cbodies[1], spVector( 50.0f, -50.0f), spVector(0.0f, 0.0f), spVector(0.0f, 1.0f), 2.0f, 0.2f);
    spWorldAddWheelJoint(&app->world, wheel[0]);
    spWorldAddWheelJoint(&app->world, wheel[1]);
    wheel[0]->enableMotor = spTrue;
    wheel[0]->motorSpeed = 10.f;
    wheel[0]->maxMotorTorque = 2000000.0f;
    wheel[1]->enableMotor = spTrue;
    wheel[1]->motorSpeed = 10.f;
    wheel[1]->maxMotorTorque = 2000000.0f;
}

spApplication* wheel()
{
    return spApplicationNew(
        "wheel constraint test app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        wheel_init, default_loop, default_main_loop,
        0);
}
//---------------------------------------------------------------------------------------------------------------------//---------------------------------------------------------------------------------------------------------------------

void
ttest_init(spApplication* app)
{
    static const spInt MAX_BODIES = 12;
    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];
    spBody* cbodies[MAX_BODIES];
    spCircle* circle[MAX_BODIES];
    spWheelJoint* wheel[MAX_BODIES];

    spFloat r = 0.8f;
    spFloat f = 0.9f;

    create_box(app, bodies+0, boxes+0, 999999.0f, spVector(  0.0f,-50.0f), 0.0f, r, f, 0.0f, spVector(10000.0f, 10.0f));
    create_box(app, bodies+1, boxes+1, 999999.0f, spVector( 75.0f, 10.0f), 0.0f, r, f, 0.0f, spVector(10.0f, 50.0f));
    create_box(app, bodies+2, boxes+2, 999999.0f, spVector(-75.0f, 10.0f), 0.0f, r, f, 0.0f, spVector(10.0f, 50.0f));

    for (spInt i = 3; i < MAX_BODIES; ++i)
    {
        spFloat p = (spFloat)i;
        spFloat p1 = p*20.0f;
        spFloat x = p1 - 80.0f;
        spFloat y = p1+10.0f;
        spFloat a = p*2.5f*p1;
        create_box(app, bodies+i, boxes+i, 10.0f, spVector(x, y), a, r, f, 1.0f, spVector(9.0f, 20.0f));
    }

    for (spInt i = 0; i < MAX_BODIES-3; ++i)
    {
        create_circle(app, cbodies+0, circle+0, spVector((spFloat)i*19.0f-25.f, i*2+150.5f), 0.0f, 6.0f, 10.0f, 0.4f, 1.0f, 1.0f);
    }
}

spApplication* ttest()
{
    return spApplicationNew(
        "ttest app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        ttest_init, default_loop, default_main_loop,
        0);
}

//---------------------------------------------------------------------------------------------------------------------//---------------------------------------------------------------------------------------------------------------------

void
gear_init(spApplication* app)
{
    static const spInt MAX_BODIES = 12;
    spBody* cbodies[MAX_BODIES];
    spCircle* circle[MAX_BODIES];
    spGearJoint* gear[MAX_BODIES];

    create_circle(app, cbodies+0, circle+0, spVector(-25.0f, 0.0f), 0.0f, 25.0f, 10.0f, 0.4f, 1.0f, 0.0f);
    create_circle(app, cbodies+1, circle+1, spVector( 25.0f, 0.0f), 0.0f, 25.0f, 10.0f, 0.4f, 1.0f, 0.0f);

    gear[0] = spGearJointNew(cbodies[0], cbodies[1], 2.0f, 1.0f);
    spWorldAddGearJoint(&app->world, gear[0]);

    struct app_data
    {
        spBody* a;
        spBody* b;
    };
    app_data* data = (app_data*)spMalloc(sizeof(data));
    data->a = cbodies[0];
    data->b = cbodies[1];
    app->data = (spLazyPointer*)data;
}

void 
gear_loop(spApplication* app)
{
    struct app_data
    {
        spBody* a;
        spBody* b;
    };
    app_data* data = (app_data*)app->data;
    spBodyApplyTorque(data->a, 5.0f);
    spWorldStep(&app->world, app->timestep);
}

spApplication* gear()
{
    return spApplicationNew(
        "gear app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        gear_init, gear_loop, default_main_loop,
        0);
}

//---------------------------------------------------------------------------------------------------------------------//---------------------------------------------------------------------------------------------------------------------

void
point_init(spApplication* app)
{
    static const spInt MAX_BODIES = 12;
    spBody* cbodies[MAX_BODIES];
    spCircle* circle[MAX_BODIES];
    spPointJoint* point[MAX_BODIES];

    spBody* bodies[MAX_BODIES];
    spPolygon* boxes[MAX_BODIES];

    create_box(app, bodies+0, boxes+0, 999999.0f, spVector(  0.0f,-50.0f), 0.0f, 0.3f, 0.9f, 0.0f, spVector(10000.0f, 10.0f));
 
    create_circle(app, cbodies+0, circle+0, spVector(8.0f, 20.0f), 0.0f, 40.0f, 30.0f, 0.4f, 0.1f, 1.0f);
    create_circle(app, cbodies+1, circle+1, spVector(10.0f,  0.0f), 0.0f, 10.0f, 10.0f, 0.4f, 0.1f, 1.0f);

    point[0] = spPointJointNew(cbodies[0], cbodies[1], spVector(0.0f, -80.0f), spVector(-10.0f, -10.0f));
    spWorldAddPointJoint(&app->world, point[0]);
}

spApplication* point()
{
    return spApplicationNew(
        "gear app",
        spViewport(1200, 1200), spFrustumUniform(100.0f),
        spVector(0.0f, -98.0f),
        20, 1.0f / 60.0f,
        point_init, default_loop, default_main_loop,
        0);
}

//---------------------------------------------------------------------------------------------------------------------//---------------------------------------------------------------------------------------------------------------------

int main()
{
    return run(point());
}