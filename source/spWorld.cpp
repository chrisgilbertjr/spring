
#include "spDebugDraw.h"
#include "spWorld.h"

static void 
initContact(spCollisionResult* result, spContact* contact, spShape* shapeA, spShape* shapeB)
{
	spMaterial* matA = &shapeA->material;
	spMaterial* matB = &shapeB->material;

    spBody* bodyA = shapeA->body;
	spBody* bodyB = shapeB->body;

    contact->count = result->count;
	contact->normal = result->normal;
	contact->friction = spMaterialComputeFriction(matA, matB);
	contact->restitution = spMaterialComputeRestitution(matA, matB);

	for (spInt i = 0; i < contact->count; i++)
	{
        contact->points[i].rA = spSub(result->pointA[i], bodyA->p);
        contact->points[i].rB = spSub(result->pointB[i], bodyB->p);
	}
}

void 
spWorldInit(spWorld* world, const spVector& gravity)
{
    world->iterations = 10;
    world->gravity = gravity;
    world->joint_list = NULL;
    world->body_list  = NULL;
    world->contact_list = NULL;
}

spWorld 
spWorldConstruct(const spVector& gravity)
{
    spWorld world;
    spWorldInit(&world, gravity);
    return world;
}

void 
spWorldStep(spWorld* world, const spFloat h)
{
    /// do broad phase collision detection
    spWorldBroadPhase(world);

    /// do narrow phase collision detection
    spWorldNarrowPhase(world);

    /// pre step the constraints
    for_each_constraint(joint, world->joint_list)
    {
        spConstraintPreStep(joint, h);
    }

    /// pre step the contacts
    for_each_contact(contact, world->contact_list)
    {
        spContactPreSolve(contact, h);
    }

    /// integrate forces and update velocity
    for_each_body(body, world->body_list)
    {
        spBodyIntegrateVelocity(body, world->gravity, h);
    }

    /// warm start the joints
    for_each_constraint(joint, world->joint_list)
    {
        spConstraintApplyCachedImpulse(joint, h);
    }

    /// pre step the contacts
    for_each_contact(contact, world->contact_list)
    {
        spContactApplyCachedImpulse(contact, h);
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
    for_each_body(body, world->body_list)
    {
        spBodyIntegratePosition(body, h);
    }

    for_each_constraint(joint, world->joint_list)
    {
        spConstraintStabilize(joint);
    }

    spWorldDraw(world);
}

void spWorldBroadPhase(spWorld* world)
{
    for_each_body(body_a, world->body_list)
    {
        for_each_body(body_b, body_a->next)
        {
            for_each_shape(shape_a, body_a->shapes)
            {
                for_each_shape(shape_b, body_b->shapes)
                {
                    spBound* ba = &shape_a->bound; ///< bound of shape a
                    spBound* bb = &shape_b->bound; ///< bound of shape b

                    /// check if the two shapes can collide (via collision filters)
                    if (spShapesCanCollide(shape_a, shape_b) == spFalse) continue;

                    /// check if the shapes AABB's overlap
                    if (spBoundBoxOverlap(*ba, *bb, body_a->xf, body_b->xf) == spFalse) continue;

                    /// they do overlap, create a contact key
                    spContactKey key = spContactKey(shape_a, shape_b);

                    /// check if the contact key is currently in the contact list
                    if (spContactKeyExists(key, world->contact_list) == spFalse)
                    {
                        /// the contact key is not in the list, create a new contact with the key
                        spContact* contact = spContactNew(key);

                        /// insert the contact into the contact list
                        spContactAdd(contact, world->contact_list);
                    };
                }
            }
        }
    }
}

void spWorldNarrowPhase(spWorld* world)
{
    spContact* contact = world->contact_list;
    while (contact != NULL)
    {
        spContactKey* key     = &contact->key; ///< contact key of the contact
        spShape*      shapeA  = key->shape_a;  ///< shape a of the contact
        spShape*      shapeB  = key->shape_b;  ///< shape b of the contact

        /// collide the two shapes
        spCollisionFunc Collide = CollideFunc[shapeA->type][shapeB->type];
        spCollisionResult result = Collide(shapeA, shapeB);
        if (result.colliding == spFalse)
        {
            /// destroy the contact
            spContact* destroy = contact;
            spContact* next = contact->next;
            spContactRemove(destroy, world->contact_list);
            spContactFree(&destroy);
            contact = next;
        }
        else
        {
            initContact(&result, contact, shapeA, shapeB);
            contact = contact->next;
        }
    }
}

void spWorldDraw(spWorld* world)
{
#ifdef SP_DEBUG_DRAW
    spBody* body_list = world->body_list;
    for_each_body(body, body_list)
    {
        for_each_shape(shape, body->shapes)
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
            if (shape->type == SP_SHAPE_SEGMENT)
            {
                spDebugDrawSegment(0, (spSegment*)shape);
            }
        }
    }
    for_each_contact(contact, world->contact_list)
    {
        //spDebugDrawContact(0, contact, contact->key.shape_b->body->xf);
    }
    spLog("\n");
#endif
}

spShape* 
spWorldTestPoint(spWorld* world, spVector point)
{
    for_each_body(body, world->body_list)
    {
        for_each_shape(shape, body->shapes)
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