
#include "spContact.h"
#include "spBody.h"

static spFloat baumgarte = 0.1f;
spFloat spSlop = 0.15f;

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
spContactInit(spContact* contact, const spContactKey key)
{
    NULLCHECK(contact);
    spContactPointInit(contact->points+0);
    spContactPointInit(contact->points+1);
    contact->key.shapeA = key.shapeA;
    contact->key.shapeB = key.shapeB;
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
spContactNew(const spContactKey key)
{
    spContact* contact = spContactAlloc();
    NULLCHECK(contact);
    spContactInit(contact, key);
    return contact;
}

void 
spContactFree(spContact** contact)
{
    NULLCHECK(*contact);
    spFree(contact);
}

void 
spContactPreSolve(spContact* contact, const spFloat h)
{
    /// get the bodies
    spBody* a = contact->key.shapeA->body;
    spBody* b = contact->key.shapeB->body;

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

        /// compute bounce bias and velocity bias (compute position constraint)
        point->bounce = spDot(relVelocity, normal) * -contact->restitution;
        point->bias = (penetration > spSlop) ? (-baumgarte * (penetration + spSlop) / h) : 0.0f;
    }
}

void 
spContactApplyCachedImpulse(spContact* contact)
{
    /// get the bodies
    spBody* a = contact->key.shapeA->body;
    spBody* b = contact->key.shapeB->body;

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

        /// reset accumulated multipliers
        point->lambdaAccumNorm = 0.0f;
        point->lambdaAccumTang = 0.0f;
    }
}

void 
spContactSolve(spContact* contact)
{
    /// get the bodies
    spBody* a = contact->key.shapeA->body;
    spBody* b = contact->key.shapeB->body;

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

spContactKey 
spContactGetKey(spContact* contact)
{
    return contact->key;
}

spContact* 
spContactGetNext(spContact* contact)
{
    return contact->next;
}

spContact* 
spContactGetPrev(spContact* contact)
{
    return contact->prev;
}

spVector 
spContactGetNormal(spContact* contact)
{
    return contact->normal;
}

spFloat 
spContactGetRestitution(spContact* contact)
{
    return contact->restitution;
}

spFloat 
spContactGetFriction(spContact* contact)
{
    return contact->friction;
}

spInt 
spContactGetPointCount(spContact* contact)
{
    return contact->count;
}