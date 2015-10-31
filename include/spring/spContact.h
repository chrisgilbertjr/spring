
#ifndef SP_CONTACT_H
#define SP_CONTACT_H

#include "spContactKey.h"
#include "spMath.h"

/// @defgroup spContact spContact
/// @{

/// penetration slop
SPRING_API extern spFloat spSlop;

typedef enum 
{
    SP_CONTACT_SHOULD_SWAP_FLAG = 1<<0,
    SP_CONTACT_DO_SWAP_FLAG     = 1<<1,
    SP_CONTACT_IS_SENSOR_FLAG   = 1<<2,
} spContactFlag;

/// a point of contact between two shapes
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

/// a contact that describes contact information between two shapes and how they react
struct spContact
{
    spContactPoint points[2]; ///< contact points
    spContactKey key;         ///< contact key containing bodies
    spContact* next;          ///< next contact in the doubly linked list
    spContact* prev;          ///< previous contact in the doubly linked list
    spVector normal;          ///< shared contact normal
    spFloat restitution;      ///< 'bounciness' of the contact
    spFloat friction;         ///< friction of the contact
    spMask flags;             ///< contacts flags
    spInt count;              ///< number of contact points
};

/// initialize a contact point
SPRING_API void spContactPointInit(spContactPoint* point);

/// initialize a contact
SPRING_API void spContactInit(spContact* contact, const spContactKey key);

/// allocate space for a new contact on the heap
SPRING_API spContact* spContactAlloc();

/// allocate and init a new contact on the heap
SPRING_API spContact* spContactNew(const spContactKey key);

/// release the memory of a contact
SPRING_API void spContactFree(spContact** contact);

/// initialize a constraint for use in the impulse solver
SPRING_API void spContactPreSolve(spContact* contact, const spFloat h);

/// warm start contacts from last frame
SPRING_API void spContactWarmStart(spContact* contact);

/// calculate and apply an impulse to each body in the contact
SPRING_API void spContactSolve(spContact* contact);

/// get the contact key of this contact
SPRING_API spContactKey spContactGetKey(spContact* contact);

/// get the next contact in the list
SPRING_API spContact* spContactGetNext(spContact* contact);

/// get the prev contact in the list
SPRING_API spContact* spContactGetPrev(spContact* contact);

/// get the contact's normal
SPRING_API spVector spContactGetNormal(spContact* contact);

/// get the contact's restitution
SPRING_API spFloat spContactGetRestitution(spContact* contact);

/// get the contact's friction
SPRING_API spFloat spContactGetFriction(spContact* contact);

/// get the contact point count
SPRING_API spInt spContactGetPointCount(spContact* contact);

/// @}

#endif