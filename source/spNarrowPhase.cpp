
#include "spDebugDraw.h"
#include "spNarrowPhase.h"
#include "spContact.h"
#include "spBody.h"

void initContact(spCollisionResult* result, spContact* contact, spShape* shapeA, spShape* shapeB)
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
        /// TODO: eliminate pen
        contact->points[i].pen = -spDot(spSub(result->pointB[i], result->pointA[i]), result->normal);
        contact->points[i].rA = spSub(result->pointA[i], bodyA->p);
        contact->points[i].rB = spSub(result->pointB[i], bodyB->p);
	}
}

spNarrowPhase
_spNarrowPhase(spContact* contact_list)
{
    return 
    { 
        spCollisionMatrix(), /// collision_matrix
        NULL        /// contact_list
    };
}

spCollisionResult
spNarrowPhaseCollide(const spNarrowPhase* narrow, const spShape* shapeA, const spShape* shapeB)
{
    spCollisionMatrix matrix = narrow->collision_matrix;

    spCollisionFunc collision_func = spCollisionQueryFunc(matrix, shapeA->type, shapeB->type);
    return collision_func(shapeA, shapeB);
}

void 
spNarrowPhaseStep(spNarrowPhase* narrow, spContact*& contact_list)
{
    spContact* contact = contact_list;
    while (contact != NULL)
    {
        spContactKey* key     = &contact->key; ///< contact key of the contact
        spShape*      shapeA  = key->shape_a;  ///< shape a of the contact
        spShape*      shapeB  = key->shape_b;  ///< shape b of the contact

        /// collide the two shapes
        spCollisionResult result = spNarrowPhaseCollide(narrow, shapeA, shapeB);
        if (result.colliding == spFalse)
        {
            /// destroy the contact
            spContact* destroy = contact;
            spContact* next = contact->next;
            spContactRemove(destroy, contact_list);
            spContactFree(destroy);
            contact = next;
        }
        else
        {
            initContact(&result, contact, shapeA, shapeB);
            contact = contact->next;
        }
    }
}
