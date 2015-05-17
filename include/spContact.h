
#ifndef SP_CONTACT_H
#define SP_CONTACT_H

#include "spContactKey.h"
#include "spMath.h"

/// @defgroup spContact spContact
/// @{

struct spContactPoint
{
#ifdef SP_DEBUG_DRAW
    spVector p;     ///< used for drawing contact points
#endif
    spVector r_a;   ///< relative velocity from body a com
    spVector r_b;   ///< relative velocity from body b com
    spFloat m_norm; ///< effective normal mass
    spFloat m_tang; ///< effective tangent mass
    spFloat La_norm;///< accumulated normal impulse multiplier
    spFloat La_tang;///< accumulated tangent impulse multiplier
    spFloat L_bias; ///< impulse bias multiplier
    spFloat v_bias; ///< velocity bias
    spFloat b_bias;
};

struct spContact
{
    static const spInt SP_MAX_CONTACT_POINTS = 2; ///< max contact points per contact
    spContactPoint points[SP_MAX_CONTACT_POINTS]; ///< contact points
    spContactKey key;
    spContact* next; ///< next contact in the doubly linked list
    spContact* prev; ///< previous contact in the doubly linked list
    spVector normal; ///< shared contact normal
    spFloat restitution; ///< 'bounciness' of the contact
    spFloat friction; ///< friction of the contact
    spInt count; ///< number of contact points
    spInt age;   ///< number of steps this contact has been alive after being out of contact
};

/// initialize a contact point
void spContactPointInit(spContactPoint* point);

/// initialize a contact
void spContactInit(spContact* contact, const spContactKey& key);

/// allocate space for a new contact on the heap
spContact* spContactAlloc();

/// allocate and init a new contact on the heap
spContact* spContactNew(const spContactKey& key);

/// release the memory of a contact
void spContactFree(spContact* contact);

/// insert a contact into the contact list
void spContactAdd(spContact* contact, spContact*& list);

/// remove a contact from the contact list
void spContactRemove(spContact* contact, spContact*& list);

/// initialize a constraint for use in the impulse solver
void spContactPreStep(spContact* contact, const spFloat h);

/// calculate and apply an impulse to each body in the contact
void spContactSolve(spContact* contact);

/// position correction to keep things more stable
void spContactStabilize(spContact* contact);

/// log contact list
void spContactLog(spContact* contact_list);

/// compute the effective mass of a contact constraint (E = J*Mi*Jt)
/// @see spConstraint.h
spFloat spContactEffectiveMass(
    const spFloat mass_invA,
    const spFloat mass_invB,
    const spFloat inertia_invA,
    const spFloat inertia_invB,
    const spVector rA,
    const spVector rB,
    const spVector norm);

#ifdef SP_DEBUG
 #define spContactPointIsSane(contact_point) _spContactPointIsSane(contact_point)
 #define spContactIsSane(contact) _spContactIsSane(contact)

 inline void _spContactPointIsSane(spContactPoint* point)
 {
     /// TODO:
 }

 inline void _spContactIsSane(spContact* contact)
 {
     /// TODO:
 }
#else
 #define spContactPointIsSane(contact_point)
 #define spContactIsSane(contact) 
#endif

/// @}

#endif