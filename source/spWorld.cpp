
#include "spDebugDraw.h"
#include "spWorld.h"

void 
spWorldInit(spWorld* world, const spVector& gravity)
{
    world->iterations = 10;
    world->gravity = gravity;
    world->joint_list = NULL;
    world->body_list  = NULL;
    world->broad_phase = spBroadPhase(0);
    world->narrow_phase = spNarrowPhase(world->broad_phase.contact_list);
    world->contact_list = NULL;
}

spWorld 
_spWorld(const spVector& gravity)
{
    spWorld world;
    spWorldInit(&world, gravity);
    return world;
}

void 
spWorldStep(spWorld* world, const spFloat h)
{
    spBody* body_list = world->body_list;

    spBroadPhaseStep(&world->broad_phase, body_list, world->contact_list);

    if (world->contact_list != NULL)
    {
        int x = 0;
    }
    spNarrowPhaseStep(&world->narrow_phase, world->contact_list);

    spContact* contact_list = world->contact_list;

    /// pre step the contacts / joints
    for_each_contact(contact, world->contact_list)
    {
        spContactPreStep(contact, h);
    }

    /// integrate forces and update velocity
    for_each_body(body, body_list)
    {
        spBodyIntegrateVelocity(body, world->gravity, h);
    }

    /// apply contact / joint impulses
    for (spInt i = 0; i < world->iterations; ++i)
    {
        for_each_contact(contact, world->contact_list)
        {
            spContactSolve(contact);
        }
    }

    /// integrate velocity and update position
    for_each_body(body, body_list)
    {
        spBodyIntegratePosition(body, h);
    }

    /// correct positions to keep things stable
    for_each_contact(contact, world->contact_list)
    {
        spContactStabilize(contact);
    }

#ifdef SP_DEBUG_DRAW
    for_each_contact(contact, world->contact_list)
    {
        spDebugDrawContact(0, contact, contact->key.shape_a->body->xf);
    }

    for_each_body(body, body_list)
    {
        for_each_shape(shape, body->shape_list)
        {
            if (shape->type == SP_SHAPE_CIRCLE)
            {
                spDebugDrawCircle(0, (spCircle*)shape, body->xf);
            }
            if (shape->type == SP_SHAPE_POLYGON)
            {
                spDebugDrawPolygon(0, (spPolygon*)shape, body->xf);
            }
            spDebugDrawBound(0, &shape->bound, body->xf);
        }
    }
#endif
}

void 
spWorldLogBrief(spWorld* world)
{
}

void 
spWorldLogBriefBodies(spWorld* world)
{
    spLog("Log Brief Bodies\n");
    spLog("----------------\n");
    spInt i = 0;
    for_each_body(body, world->body_list)
    {
        spLog(" -> %p ", body);
        if (i % 6 > 4 || body->next == NULL) { spLog("\n"); }
        ++i;
    }
    spLog("\n");
}

void 
spWorldLogDetail(spWorld* world)
{
}