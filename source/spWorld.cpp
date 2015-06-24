
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

    /// pre step the constraints
    for_each_constraint(joint, world->joint_list)
    {
        spConstraintPreStep(joint, h);
    }

    /// pre step the contacts
    for_each_contact(contact, world->contact_list)
    {
        spContactPreStep(contact, h);
    }

    /// integrate forces and update velocity
    for_each_body(body, body_list)
    {
        spBodyIntegrateVelocity(body, world->gravity, h);
    }

    /// warm start the joints
    for_each_constraint(joint, world->joint_list)
    {
        spConstraintApplyCachedImpulse(joint, h);
    }

    /// apply contact / joint impulses
    for (spInt i = 0; i < world->iterations; ++i)
    {
        for_each_constraint(joint, world->joint_list)
        {
            spConstraintSolve(joint);
        }

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

    for_each_constraint(joint, world->joint_list)
    {
        spConstraintStabilize(joint);
    }

    /// correct positions to keep things stable
    for_each_contact(contact, world->contact_list)
    {
        spContactStabilize(contact);
    }
    spWorldDraw(world);
}

void spWorldDraw(spWorld* world)
{
    spBody* body_list = world->body_list;
#ifdef SP_DEBUG_DRAW
    for_each_body(body, body_list)
    {
        for_each_shape(shape, body->shape_list)
        {
            spDebugDrawBound(0, &shape->bound, body->xf);
            if (shape->type == SP_SHAPE_CIRCLE)
            {
                spDebugDrawCircle(0, (spCircle*)shape, body->xf);
            }
            if (shape->type == SP_SHAPE_POLYGON)
            {
                spDebugDrawPolygon(0, (spPolygon*)shape, body->xf);
            }
        }
    }
    for_each_contact(contact, world->contact_list)
    {
        spDebugDrawContact(0, contact, contact->key.shape_b->body->xf);
    }
    spLog("\n");
#endif
}

spShape* 
spWorldTestPointAgainstShapes(spWorld* world, spVector point)
{
    for_each_body(body, world->body_list)
    {
        for_each_shape(shape, body->shape_list)
        {
            if (spShapeTestPoint(shape, point))
            {
                return shape;
            }
        }
    }
    return NULL;
}

void 
spWorldAddBody(spWorld* world, spBody* body)
{
    SP_LINKED_LIST_PREPEND(spBody, body, world->body_list);
}

void 
spWorldAddDistanceJoint(spWorld* world, spDistanceJoint* joint)
{
    spConstraint* constraint = &joint->base_class;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldAddRopeJoint(spWorld* world, spRopeJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldAddMotorJoint(spWorld* world, spMotorJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldAddSpringJoint(spWorld* world, spSpringJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldAddAngularSpringJoint(spWorld* world, spAngularSpringJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldAddWheelJoint(spWorld* world, spWheelJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldAddGearJoint(spWorld* world, spGearJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void spWorldAddPointJoint(spWorld* world, spPointJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void spWorldAddMouseJoint(spWorld* world, spMouseJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->joint_list);
}

void 
spWorldRemoveMouseJoint(spWorld* world, spMouseJoint* joint)
{
    spConstraint* constraint = &joint->constraint;
    SP_LINKED_LIST_REMOVE(spConstraint, constraint, world->joint_list);
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