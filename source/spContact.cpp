
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
    point->L_bias = 0.0f;
    point->v_bias = 0.0f;
    point->bounce = 0.0f;
    point->bias   = 0.0f;
    point->pen    = 0.0f;
}

void 
spContactInit(spContact* contact, const spContactKey& key)
{
    for (spInt i = 0; i < spContact::SP_MAX_CONTACT_POINTS; ++i)
    {
        spContactPointInit(contact->points + i);
    }
    contact->key.shape_a = key.shape_a;
    contact->key.shape_b = key.shape_b;
    contact->next        = NULL;
    contact->prev        = NULL;
    contact->normal      = spVectorZero();
    contact->restitution = 0.0f;
    contact->friction    = 0.0f;
    contact->count       = 0;
    contact->age         = 0;
    contact->pen         = 0.0f;
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
spContactFree(spContact* contact)
{
    spFree(contact);
}

void 
spContactAdd(spContact* contact, spContact*& list)
{
    //SP_LINKED_LIST_PREPEND(spContact, contact, list);
    spAssert(contact != NULL, "Error: contact is NULL, cannot add contact");

    /// insert into the front of the list
    if (list == NULL)
    {
        contact->next = NULL;
    }
    else
    {
        list->prev = contact;
        contact->next = list;
    }
    contact->prev = NULL;
    list = contact;
}

void 
spContactRemove(spContact* contact, spContact*& list)
{
    /// check if the list is empty
    spAssert(list != NULL, "Error: contact list is NULL, cannot remove contact");
    spAssert(contact != NULL, "Error: contact is NULL, cannot remove contact");

    spContact* c_prev = contact->prev;
    spContact* c_next = contact->next;

    contact->prev = NULL;
    contact->next = NULL;

    /// the contact is in the middle of the list
    if (c_prev != NULL && c_next != NULL)
    {
        c_prev->next = c_next;
        c_next->prev = c_prev;
    }

    /// the contact is the only contact in the list
    else if (c_prev == NULL && c_next == NULL)
    {
        list = NULL;
    }

    /// the contact is at the beginning of the list
    else if (c_prev == NULL)
    {
        list = c_next;
        c_next->prev = NULL;
    }

    /// the contact is at the end of the list
    else if (c_next == NULL)
    {
        c_prev->next = NULL;
    }

    else
    {
        spAssert(false, "removing contact, shouldnt reach this...");
    }
}

void 
spContactPreStep(spContact* contact, const spFloat h)
{
    spBody* a = contact->key.shape_a->body;   /// rigid body a
    spBody* b = contact->key.shape_b->body;   /// rigid body b

    spInt    points  = contact->count;
    spVector normal  = contact->normal;
    spVector tangent = spSkew(normal);

    for (spInt i = 0; i < points; ++i)
    {
        spContactPoint* point = &contact->points[i];
        spFloat invMass = a->m_inv + b->m_inv;

        {
            /// compute non penetration effective mass
            spFloat cA = spCross(point->rA, normal);
            spFloat cB = spCross(point->rB, normal);
            point->eMassNorm = invMass + a->i_inv * cA * cA + b->i_inv * cB * cB;
            point->eMassNorm = point->eMassNorm ? 1.0f / point->eMassNorm : 0.0f;
        }{
            /// compute friction effective mass
            spFloat cA = spCross(point->rA, tangent);
            spFloat cB = spCross(point->rB, tangent);
            point->eMassTang = invMass + a->i_inv * cA * cA + b->i_inv * cB * cB;
            point->eMassTang = point->eMassTang ? 1.0f / point->eMassTang : 0.0f;
        }

        /// compute relative velocity
        spVector rvA = spAdd(a->v, spCross(a->w, point->rA));
        spVector rvB = spAdd(b->v, spCross(b->w, point->rB));
        spVector relVelocity = spSub(rvB, rvA);

        /// compute penetration and set position slop
        spFloat pen = -spDot(spAdd(spSub(point->rB, point->rA), spSub(b->p, a->p)), normal);
        static const spFloat slop = -0.55f;

        /// compute bounce bias and velocity bias
        point->bounce = spDot(relVelocity, normal) * -contact->restitution;
        point->bias = (pen > slop) ? (-0.2f * (pen + slop) / h) : 0.0f;

        /// reset accumulated multipliers
        point->lambdaAccumNorm = 0.0f;
        point->lambdaAccumTang = 0.0f;
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

        spBodyApplyImpulse(a, point->rA, impulseA);
        spBodyApplyImpulse(b, point->rB, impulseB);
    }
}

void 
spContactStabilize(spContact* contact)
{
    /// stabilize position
    spVector normal = contact->normal;           /// contact normal
    spBody* body_a = contact->key.shape_a->body; /// rigid body a
    spBody* body_b = contact->key.shape_b->body; /// rigid body b
    spFloat mia = body_a->m_inv;                 /// inv mass of body a
    spFloat mib = body_b->m_inv;                 /// inv mass of body b

    const static spFloat slop = 1.5f;
    const static spFloat perc = 0.40f;
    if (contact->pen < slop) return;

    spFloat im = mia + mib;
    if (im != 0.0f)
    {
        spVector P = spMult((spMax(contact->pen - slop, 0.0f) / im), spMult(normal, perc));
    	body_a->p = spSub(body_a->p, spMult(P, mia));
    	body_b->p = spAdd(body_b->p, spMult(P, mib));
        __spBodyUpdateTransform(body_a);
    	__spBodyUpdateTransform(body_b);
    }
}

void 
spContactLog(spContact* contact_list)
{
    /// TODO:
}

void 
spContactLogList(spContact* contact_list)
{
    for_each_contact(contact, contact_list)
    {
        spContactLog(contact);
    }
}

spFloat spContactEffectiveMass(
    const spFloat mass_invA,
    const spFloat mass_invB,
    const spFloat inertia_invA,
    const spFloat inertia_invB,
    const spVector rA,
    const spVector rB,
    const spVector norm)
{
    spFloat rcnA = spCross(rA, norm);
    spFloat ksbA = mass_invA + inertia_invA * rcnA * rcnA;

    spFloat rcnB = spCross(rB, norm);
    spFloat ksbB = mass_invB + inertia_invB * rcnB * rcnB;

    return ksbA + ksbB;
}
