
#include "spContact.h"
#include "spSolver.h"
#include "spBody.h"

void 
spContactPointInit(spContactPoint* point)
{
    point->rA = spVectorZero();
    point->rB = spVectorZero();
    point->lambdaAccumNorm = 0.0f;
    point->lambdaAccumTang = 0.0f;
    point->eMassNorm = 0.0f;
    point->eMassTang = 0.0f;
    point->bounce = 0.0f;
    point->bias   = 0.0f;
}

void 
spContactInit(spContact* contact, const spContactKey& key)
{
    spContactPointInit(contact->points+0);
    spContactPointInit(contact->points+1);
    contact->key.shape_a = key.shape_a;
    contact->key.shape_b = key.shape_b;
    contact->next        = NULL;
    contact->prev        = NULL;
    contact->normal      = spVectorZero();
    contact->restitution = 0.0f;
    contact->friction    = 0.0f;
    contact->count       = 0;
}

spContact* 
spContactAlloc()
{
    return (spContact*)spMalloc(sizeof(spContact));
}

spContact* 
spContactNew(const spContactKey& key)
{
    spContact* contact = spContactAlloc();
    spContactInit(contact, key);
    return contact;
}

void 
spContactFree(spContact** contact)
{
    spFree(contact);
}

void 
spContactAdd(spContact* contact, spContact*& list)
{
    SP_LINKED_LIST_PREPEND(spContact, contact, list);
}

void 
spContactRemove(spContact* contact, spContact*& list)
{
    SP_LINKED_LIST_REMOVE(spContact, contact, list);
}

void 
spContactPreSolve(spContact* contact, const spFloat h)
{
    /// get the bodies
    spBody* a = contact->key.shape_a->body;
    spBody* b = contact->key.shape_b->body;

    spInt    points  = contact->count;
    spVector normal  = contact->normal;
    spVector tangent = spSkew(normal);

    for (spInt i = 0; i < points; ++i)
    {
        spContactPoint* point = &contact->points[i];
        spFloat invMass = a->mInv + b->mInv;

        {
            /// compute non penetration effective mass
            spFloat cA = spCross(point->rA, normal);
            spFloat cB = spCross(point->rB, normal);
            point->eMassNorm = invMass + a->iInv * cA * cA + b->iInv * cB * cB;
            point->eMassNorm = point->eMassNorm ? 1.0f / point->eMassNorm : 0.0f;
        }{
            /// compute friction effective mass
            spFloat cA = spCross(point->rA, tangent);
            spFloat cB = spCross(point->rB, tangent);
            point->eMassTang = invMass + a->iInv * cA * cA + b->iInv * cB * cB;
            point->eMassTang = point->eMassTang ? 1.0f / point->eMassTang : 0.0f;
        }

        /// compute relative velocity
        spVector rvA = spAdd(a->v, spCross(a->w, point->rA));
        spVector rvB = spAdd(b->v, spCross(b->w, point->rB));
        spVector relVelocity = spSub(rvB, rvA);

        /// compute penetration and set position slop
        spFloat penetration = -spDot(spAdd(spSub(point->rB, point->rA), spSub(b->p, a->p)), normal);
        static const spFloat slop = -0.55f;

        /// compute bounce bias and velocity bias
        point->bounce = spDot(relVelocity, normal) * -contact->restitution;
        point->bias = (penetration > slop) ? (-0.2f * (penetration + slop) / h) : 0.0f;

        /// reset accumulated multipliers
        point->lambdaAccumNorm = 0.0f;
        point->lambdaAccumTang = 0.0f;
    }
}

void 
spContactApplyCachedImpulse(spContact* contact, const spFloat h)
{
    /// get the bodies
    spBody* a = contact->key.shape_a->body;
    spBody* b = contact->key.shape_b->body;

    for (spInt i = 0; i < contact->count; ++i)
    {
        spContactPoint* point = contact->points+i;

        /// compute the impulses
        spVector impulse = spVector(point->lambdaAccumNorm, point->lambdaAccumTang);
        spVector impulseA = spRotate(contact->normal, impulse);
        spVector impulseB = spNegate(impulseA);

        /// apply the impulses
        spBodyApplyImpulse(a, point->rA, impulseA);
        spBodyApplyImpulse(b, point->rB, impulseB);
    }
}

void 
spContactSolve(spContact* contact)
{
    spBody* a = contact->key.shape_a->body;
    spBody* b = contact->key.shape_b->body;

    spInt    points  = contact->count;
    spVector normal  = contact->normal;
    spVector tangent = spSkew(normal);

    for (spInt i = 0; i < points; ++i)
    {
        spContactPoint* point = &contact->points[i];

        /// compute relative velocity between the two bodies
        spVector rvA = spAdd(a->v, spCross(a->w, point->rA));
        spVector rvB = spAdd(b->v, spCross(b->w, point->rB));
        spVector relVelocity = spSub(rvB, rvA);

        /// lagrange multipliers used to compute the impulse
        spFloat impulseNorm, impulseTang;

        {
            /// solve non-penetration constraint
            /// get effective mass and solve the velocity constraint
            spFloat eMass = point->eMassNorm;
        	spFloat Cdot  = spDot(relVelocity, normal);

            /// compute lagrange multiplier
        	spFloat lambdaOld = point->lambdaAccumNorm;
        	spFloat lambda    = -(Cdot + point->bounce + point->bias) * eMass;

            /// accumulate the multiplier and compute the impulse
        	point->lambdaAccumNorm = spMax(lambdaOld + lambda, 0.0f);
            impulseNorm = point->lambdaAccumNorm - lambdaOld;
        } {
            /// solve friction constraint
            /// get effective mass and solve the velocity constraint
            spFloat eMass = point->eMassTang;
        	spFloat Cdot  = spDot(relVelocity, tangent);

            /// compute lagrange multiplier
        	spFloat lambdaMax = contact->friction * point->lambdaAccumNorm;
        	spFloat lambdaOld = point->lambdaAccumTang;
        	spFloat lambda    = -Cdot * eMass;

            /// accumulate the multiplier and compute the impulse
        	point->lambdaAccumTang = spClamp(lambdaOld + lambda, -lambdaMax, lambdaMax);
            impulseTang = point->lambdaAccumTang - lambdaOld;
        }

        /// compute the body impulses
        spVector impulseB = spRotate(normal, spVector(impulseNorm, impulseTang));
        spVector impulseA = spNegate(impulseB);

        /// apply the impulses
        spBodyApplyImpulse(a, point->rA, impulseA);
        spBodyApplyImpulse(b, point->rB, impulseB);
    }
}