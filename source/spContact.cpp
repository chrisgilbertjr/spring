
#include "spContact.h"
#include "spSolver.h"
#include "spBody.h"

void 
spContactPointInit(spContactPoint* point)
{
    point->r_a     = spVectorZero();
    point->r_b     = spVectorZero();
    point->m_norm  = 0.0f;
    point->m_tang  = 0.0f;
    point->La_norm = 0.0f;
    point->La_tang = 0.0f;
    point->L_bias  = 0.0f;
    point->v_bias  = 0.0f;
    point->b_bias  = 0.0f;
}

void 
spContactInit(spContact* contact, const spContactKey& key)
{
    for (spInt i = 0; i < spContact::SP_MAX_CONTACT_POINTS; ++i)
    {
        spContactPointInit(contact->points + i);
    }
    spContactPointInit(contact->points);
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
    spInt point_count = contact->count; /// number of contact points
    spVector normal = contact->normal;  /// normal of the contact
    spBody* body_a = contact->key.shape_a->body;   /// rigid body a
    spFloat ima = body_a->m_inv;        /// inverse mass of body a
    spFloat iia = body_a->i_inv;        /// inverse inertia of body a
    spBody* body_b = contact->key.shape_b->body;   /// rigid body b
    spFloat imb = body_b->m_inv;        /// inverse mass of body b
    spFloat iib = body_b->i_inv;        /// inverse inertia of body b

    for (spInt i = 0; i < point_count; ++i)
    {
        spContactPoint* point = &contact->points[i];
        spVector ra = point->r_a;
        spVector rb = point->r_b;

        /// TODO: optimize
        point->m_norm = spContactEffectiveMass(ima, imb, iia, iib, ra, rb, normal);
        point->m_tang = spContactEffectiveMass(ima, imb, iia, iib, ra, rb, spSkew(normal));
        point->m_norm = point->m_norm ? 1.0f / point->m_norm : 0.0f;
        point->m_tang = point->m_tang ? 1.0f / point->m_tang : 0.0f;

        spVector rv = spRelativeVelocity(body_a->v, body_b->v, body_a->w, body_b->w, ra, rb);
        point->b_bias = spDot(rv, normal) * -contact->restitution;
        point->L_bias = 0.0f;
    }
}

void 
spContactSolve(spContact* contact)
{
    /// @see spConstraint
    spInt contact_points = contact->count; /// number of contact points
    spVector normal = contact->normal;     /// contact normal
    spVector tangent = spSkew(normal);     /// contact tangent
    spBody* body_a = contact->key.shape_a->body;      /// rigid body a
    spFloat mia = body_a->m_inv;           /// inv mass of body a
    spFloat iia = body_a->i_inv;           /// inv inertia of body a
    spBody* body_b = contact->key.shape_b->body;      /// rigid body b
    spFloat mib = body_b->m_inv;           /// inv mass of body b
    spFloat iib = body_b->i_inv;           /// inv inertia of body b

    /// ** friction constraint **
    /// .
    /// C = dot(vpB - vpA, t)
    /// J = [ -t, -rA x t, t, rB x t ]
    /// rA = center of mass of bodyA to the contact point
    /// rB = center of mass of bodyB to the contact point
    /// t = contact tangent
    ///
    /// ** non-penetration constraint **
    /// .
    /// C = dot(vpB - vpA, n)
    /// J = [ -n, -rA x n, n, rB x n ]
    /// rA = center of mass of bodyA to the contact point
    /// rB = center of mass of bodyB to the contact point
    /// n = contact normal
    for (spInt i = 0; i < contact_points; ++i)
    {
        spContactPoint point = contact->points[i]; /// contact point
        spVector ra = point.r_a;
        spVector rb = point.r_b;

        /// get the precomputed effective normal mass
        /// E = J*Mi*Jt
        /// Ei = 1 / E
        spFloat Ein = point.m_norm;
        spFloat Eit = point.m_tang;

        /// compute J*V
        /// This can be simplified to:
        /// J*V = dot(vB + cross(wB, rB) - vA - cross(wA, rA), normal);
        spVector rv = spRelativeVelocity(body_a->v, body_b->v, body_a->w, body_b->w, ra, rb);
        spFloat JVn = spDot(rv, normal);
        spFloat JVt = spDot(rv, tangent);

        /// compute the lagrange multiplier
        /// L = -(J*V-b) / E
        //spFloat Ln = -(point.b_bias + JVn) * Ein;
        /// clamp the accumulated impulses

        spFloat Ln = -Ein * (JVn - point.b_bias);
        spFloat LnOld = point.La_norm;
        point.La_norm = spMax(LnOld + Ln, 0.0f);

        spFloat Lt = -(JVt) * Eit; /// tangent constraint has no velocity bias
        spFloat LtMax = contact->friction * point.La_norm;
        spFloat LtOld = point.La_tang;
        point.La_tang = spClamp(LtOld + Lt, -LtMax, LtMax);

        /// calculate the constraint force and apply the impulse
        /// F = L * Jt
        /// V += Mi * F
        ///
        /// vA += miA * lA;
        /// vB += miB * lB;
        /// wA += iiA * aA;
        /// wB += iiB * aB;

        spVector Pt = spMult(point.La_tang - LtOld, tangent); /// tangent impulse
        body_a->v  = spSub(body_a->v, spMult(Pt, mia));
        body_b->v  = spAdd(body_b->v, spMult(Pt, mib));
        body_a->w -= iia * spCross(ra, Pt);
        body_b->w += iib * spCross(rb, Pt);

        spVector Pn = spMult(point.La_norm - LnOld, normal); /// normal impulse
        body_a->v  = spSub(body_a->v, spMult(Pn, mia));
        body_b->v  = spAdd(body_b->v, spMult(Pn, mib));
        body_a->w -= iia * spCross(ra, Pn);
        body_b->w += iib * spCross(rb, Pn);
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

    const static spFloat slop = 0.15f;
    const static spFloat perc = 0.10f;
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
