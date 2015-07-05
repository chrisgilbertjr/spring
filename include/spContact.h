
#ifndef SP_CONTACT_H
#define SP_CONTACT_H

#include "spContactKey.h"
#include "spMath.h"

/// @defgroup spContact spContact
/// @{

struct spContactPoint
{
    spVector rA;             ///< relative velocity from body a com
    spVector rB;             ///< relative velocity from body b com
    spFloat eMassNorm;       ///< effective normal mass
    spFloat eMassTang;       ///< effective tangent mass
    spFloat lambdaAccumNorm; ///< accumulated normal impulse multiplier
    spFloat lambdaAccumTang; ///< accumulated tangent impulse multiplier
    spFloat bounce;          ///< bounce bias based on restitution
    spFloat bias;            ///< baumgarte velocity bias
};

struct spContact
{
    spContactPoint points[2]; ///< contact points
    spContactKey key;         ///< contact key containing bodies
    spContact* next;          ///< next contact in the doubly linked list
    spContact* prev;          ///< previous contact in the doubly linked list
    spVector normal;          ///< shared contact normal
    spFloat restitution;      ///< 'bounciness' of the contact
    spFloat friction;         ///< friction of the contact
    spInt count;              ///< number of contact points
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
void spContactFree(spContact** contact);

/// insert a contact into the contact list
void spContactAdd(spContact* contact, spContact*& list);

/// remove a contact from the contact list
void spContactRemove(spContact* contact, spContact*& list);

/// initialize a constraint for use in the impulse solver
void spContactPreSolve(spContact* contact, const spFloat h);

/// warm start contacts from last frame
void spContactApplyCachedImpulse(spContact* contact, const spFloat h);

/// calculate and apply an impulse to each body in the contact
void spContactSolve(spContact* contact);

/// @}

#endif