
#include "spConstraint.h"
#include "spDebugDraw.h"
#include "spWorld.h"

/// static funcs

static void 
initContact(spCollisionResult* result, spContact* contact, spShape* shapeA, spShape* shapeB)
{
    /// get the materials
	spMaterial* matA = &shapeA->material;
	spMaterial* matB = &shapeB->material;

    /// get the bodies
    spBody* bodyA = shapeA->body;
	spBody* bodyB = shapeB->body;

    /// init the contact info
    contact->count = result->count;
	contact->normal = result->normal;
    contact->friction = spsqrt(shapeA->material.friction * shapeB->material.friction);
    contact->restitution = spMax(shapeA->material.restitution, shapeB->material.restitution);

    /// get rel velocity of contact points
	for (spInt i = 0; i < contact->count; i++)
	{
        contact->points[i].rA = spSub(result->pointA[i], bodyA->p);
        contact->points[i].rB = spSub(result->pointB[i], bodyB->p);
	}
}

static void
addContact(spWorld* world, spContact* contact)
{
    SP_LINKED_LIST_PREPEND(spContact, contact, world->contactList);
}

static void 
destroyContact(spWorld* world, spContact** destroy)
{
    spContact* contact = *destroy;
    SP_LINKED_LIST_REMOVE(spContact, contact, world->contactList);
    spContactFree(destroy);
}

/// world functions

void 
spWorldInit(spWorld* world, spInt iterations, spVector gravity)
{
    world->iterations = iterations;
    world->gravity = gravity;
    world->jointList = NULL;
    world->bodyList  = NULL;
    world->contactList = NULL;
}

void 
spWorldDestroy(spWorld* world)
{
    /// destroy all contacts
    spContact* contact = world->contactList;
    while(contact)
    {
        spContact* next = contact->next;
        destroyContact(world, &contact);
        contact = next;
    }

    /// destroy all bodies
    spBody* body = world->bodyList;
    while(body)
    {
        spBody* next = body->next;
        spBodyDestroy(&body);
        body = next;
    }
    world->bodyList = NULL;

    /// destroy all constraints
    spConstraint* constraint = world->jointList;
    while(constraint)
    {
        spConstraint* next = constraint->next;
        spConstraintFree(constraint);
        constraint = next;
    }
    world->jointList = NULL;
}

spWorld 
spWorldConstruct(spInt iterations, spVector gravity)
{
    spWorld world;
    spWorldInit(&world, iterations, gravity);
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
    for_each_constraint(joint, world->jointList)
    {
        spConstraintPreSolve(joint, h);
    }

    /// pre step the contacts
    for_each_contact(contact, world->contactList)
    {
        spContactPreSolve(contact, h);
    }

    /// integrate forces and update velocity
    for_each_body(body, world->bodyList)
    {
        spBodyIntegrateVelocity(body, world->gravity, h);
    }

    /// warm start the joints
    for_each_constraint(joint, world->jointList)
    {
        spConstraintApplyCachedImpulse(joint);
    }

    /// pre step the contacts
    for_each_contact(contact, world->contactList)
    {
        spContactApplyCachedImpulse(contact);
    }

    /// apply contact / joint impulses
    for (spInt i = 0; i < world->iterations; ++i)
    {
        for_each_constraint(joint, world->jointList)
        {
            spConstraintSolve(joint);
        }

        for_each_contact(contact, world->contactList)
        {
            spContactSolve(contact);
        }
    }

    /// integrate velocity and update position
    for_each_body(body, world->bodyList)
    {
        spBodyIntegratePosition(body, h);
    }

    spWorldDraw(world);
}

void spWorldBroadPhase(spWorld* world)
{
    for_each_body(body_a, world->bodyList) { for_each_body(body_b, body_a->next)
    {
        for_each_shape(shape_a, body_a->shapes) { for_each_shape(shape_b, body_b->shapes)
        {
            /// bounding boxes of shapeA and shapeB
            spBound* ba = &shape_a->bound;
            spBound* bb = &shape_b->bound;

            /// check if the two shapes can collide (via collision filters)
            if (spShapesCanCollide(shape_a, shape_b) == spFalse) continue;

            /// check if the shapes AABB's overlap
            if (spBoundBoxOverlap(ba, bb, &body_a->xf, &body_b->xf) == spFalse) continue;

            /// they do overlap, create a contact key
            spContactKey key = spContactKeyConstruct(shape_a, shape_b);

            /// check if the contact key is currently in the contact list
            if (spContactKeyExists(key, world->contactList) == spFalse)
            {
                /// the contact key is not in the list, create a new contact with the key
                spContact* contact = spContactNew(key);

                /// insert the contact into the contact list
                addContact(world, contact);
            }
        }}
    }}
}

void spWorldNarrowPhase(spWorld* world)
{
    spContact* contact = world->contactList;
    while (contact != NULL)
    {
        /// get the contact key and shapes to collide
        spContactKey* key     = &contact->key;
        spShape*      shapeA  = key->shapeA;
        spShape*      shapeB  = key->shapeB;

        /// collide the two shapes
        spCollisionFunc Collide = CollideFunc[shapeA->type][shapeB->type];
        spCollisionResult result = Collide(shapeA, shapeB);

        /// check if they are colliding
        if (result.colliding == spFalse)
        {
            /// destroy the contact
            spContact* destroy = contact;
            spContact* next = contact->next;
            destroyContact(world, &destroy);
            contact = next;
        }

        /// they are colliding, init the contact with the collision result
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
    spBody* body_list = world->bodyList;
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
#endif
}

spShape* 
spWorldTestPoint(spWorld* world, spVector point)
{
    for_each_body(body, world->bodyList)
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
    SP_LINKED_LIST_PREPEND(spBody, body, world->bodyList);
    body->world = world;
}

void 
spWorldRemoveBody(spWorld* world, spBody* body)
{
    SP_LINKED_LIST_REMOVE(spBody, body, world->bodyList);
    body->world = NULL;
}

void 
spWorldAddConstraint(spWorld* world, spConstraint* constraint)
{
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->jointList);
}

void 
spWorldRemoveConstraint(spWorld* world, spConstraint* constraint)
{
    SP_LINKED_LIST_REMOVE(spConstraint, constraint, world->jointList);
}

spConstraint* 
spWorldGetJointList(spWorld* world)
{
    return world->jointList;
}

spContact* 
spWorldGetContactList(spWorld* world)
{
    return world->contactList;
}

spBody* 
spWorldGetBodyList(spWorld* world)
{
    return world->bodyList;
}

spVector 
spWorldGetGravity(spWorld* world)
{
    return world->gravity;
}

spInt 
spWorldGetIterations(spWorld* world)
{
    return world->iterations;
}

void 
spWorldSetGravity(spWorld* world, spVector gravity)
{
    world->gravity = gravity;
}

void 
spWorldSetIterations(spWorld* world, spInt iterations)
{
    world->iterations = iterations;
}