
#include "spWorld.h"
#include "spConstraint.h"
#include "spCollision.h"
#include "spContact.h"
#include "spBody.h"

/// for each iters
#define foreach_constraint(joint, initializer) for (spConstraint* joint = initializer; joint != NULL; joint = joint->next)
#define foreach_contact(contact, initializer) for (spContact* contact = initializer; contact != NULL; contact = contact->next)
#define foreach_shape(shape, initializer) for (spShape* shape = initializer; shape; shape = shape->next)
#define foreach_body(body, initializer) for (spBody* body = initializer; body; body = body->next)

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
    contact->friction = spsqrt(matA->friction * matB->friction);
    contact->restitution = spMax(matA->restitution, matB->restitution);

    /// get rel velocity of contact points
	for (spInt i = 0; i < contact->count; i++)
	{
        contact->points[i].rA = spvSub(result->pointA[i], bodyA->p);
        contact->points[i].rB = spvSub(result->pointB[i], bodyB->p);
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
    world->sweepAndPrune = spSapConstruct();
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

    /// destroy all constraints
    spConstraint* constraint = world->jointList;
    while(constraint)
    {
        spConstraint* next = constraint->next;
        spConstraintFree(&constraint);
        constraint = next;
    }

    world->contactList = NULL;
    world->jointList = NULL;
    world->bodyList = NULL;
    spSapDestroy(&world->sweepAndPrune);
    int x = 0;
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
    //spWorldBroadPhaseSAP(world);
    spWorldBroadPhaseBruteForce(world);

    /// do narrow phase collision detection
    spWorldNarrowPhase(world);

    /// pre step the constraints
    foreach_constraint(joint, world->jointList)
    {
        joint->funcs.preSolve(joint, h);
    }

    /// pre step the contacts
    foreach_contact(contact, world->contactList)
    {
        spContactPreSolve(contact, h);
    }

    /// integrate forces and update velocity
    foreach_body(body, world->bodyList)
    {
        spBodyIntegrateVelocity(body, world->gravity, h);
    }

    /// warm start the joints
    foreach_constraint(joint, world->jointList)
    {
        joint->funcs.warmStart(joint);
    }

    /// pre step the contacts
    foreach_contact(contact, world->contactList)
    {
        spContactWarmStart(contact);
    }

    /// apply contact / joint impulses
    for (spInt i = 0; i < world->iterations; ++i)
    {
        foreach_constraint(joint, world->jointList)
        {
            joint->funcs.solve(joint);
        }

        foreach_contact(contact, world->contactList)
        {
            spContactSolve(contact);
        }
    }

    /// integrate velocity and update position
    foreach_body(body, world->bodyList)
    {
        spBodyIntegratePosition(body, h);
    }
}

void 
spWorldBroadPhaseSAP(spWorld* world)
{
    spSap* sap = &world->sweepAndPrune;

    /// update the sap boxes to world space
    spSapUpdate(sap);

    /// quick sort the boxes to sort them spatially
    spSapSort(sap);

    /// check for interval overlap and if have potential for collision
    spInt count = sap->count;
    spSapBox** boxes = sap->boxes;
    for (spInt i = 0; i < count; ++i)
    {
        spShape* shapeA = boxes[i]->shape;
        for (spInt j = i + 1; j < count; ++j)
        {
            spShape* shapeB = boxes[j]->shape;

            /// check if the intervals overlap
            if (spIntervalsDontOverlap(boxes, i, j))
            {
                break;
            }

            /// check if the two shapes can collide
            if (spShapesCanCollide(shapeA, shapeB) == spFalse) continue;

            /// check if the two boxes overlap
            if (spBoxesOverlap(boxes[i], boxes[j]) == spFalse) continue;

            /// they do overlap, create a contact key
            spContactKey key = spContactKeyConstruct(shapeA, shapeB);

            /// check if the contact key is currently in the contact list
            if (spContactKeyExists(key, world->contactList) == spFalse && 
               (spBodyGetType(shapeA->body) != SP_BODY_STATIC || 
                spBodyGetType(shapeB->body) != SP_BODY_STATIC))
            {
                /// the contact key is not in the list, create a new contact with the key
                spContact* contact = spContactNew(key);

                /// insert the contact into the contact list
                addContact(world, contact);
            }
        }
    }
}

void 
spWorldBroadPhaseBruteForce(spWorld* world)
{
    foreach_body(body_a, world->bodyList) { foreach_body(body_b, body_a->next)
    {
        foreach_shape(shape_a, body_a->shapes) { foreach_shape(shape_b, body_b->shapes)
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
            if (spContactKeyExists(key, world->contactList) == spFalse && 
               (body_a->type != SP_BODY_STATIC || body_b->type != SP_BODY_STATIC))
            {
                /// the contact key is not in the list, create a new contact with the key
                spContact* contact = spContactNew(key);

                /// insert the contact into the contact list
                addContact(world, contact);
            }
        }}
    }}
}

void 
spWorldNarrowPhase(spWorld* world)
{
    spContact* contact = world->contactList;
    spInt i = 0;
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

spShape* 
spWorldTestPoint(spWorld* world, spVector point)
{
    foreach_body(body, world->bodyList)
    {
        foreach_shape(shape, body->shapes)
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

    foreach_shape(shape, body->shapes)
    {
        spSapInsert(&world->sweepAndPrune, shape);
    }
}

void 
spWorldRemoveBody(spWorld* world, spBody* body)
{
    SP_LINKED_LIST_REMOVE(spBody, body, world->bodyList);
    body->world = NULL;

    spShape* shape = body->shapes;
    if (shape != NULL)
    {
        while(shape)
        {
            if (shape->next == NULL)
            {
                break;
            }
            else
            {
                shape = shape->next;
            }
        }
    }

    while (shape)
    {
        spSapRemove(&world->sweepAndPrune, shape);
        if (shape->prev == NULL)
        {
            break;
        }
        shape = shape->prev;
    }
    int x = 0;
}

void 
spWorldAddConstraint(spWorld* world, spConstraint* constraint)
{
    SP_LINKED_LIST_PREPEND(spConstraint, constraint, world->jointList);
    constraint->world = world;
}

void 
spWorldRemoveConstraint(spWorld* world, spConstraint** constraint)
{
    spConstraint* c = *constraint;
    SP_LINKED_LIST_REMOVE(spConstraint, c, world->jointList);
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