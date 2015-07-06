
#ifndef SP_CONTACT_KEY_H
#define SP_CONTACT_KEY_H

#include "spShape.h"

/// @defgroup spContactKey spContactKey
/// @{

/// used for persistent contacts
struct spContactKey
{
    spShape* shapeA;
    spShape* shapeB;
};

/// initialize a contact key with two shapes
void spContactKeyInit(spContactKey* key, spShape* shapeA, spShape* shapeB);

/// 'faked' constructor for stack allocation
spContactKey spContactKeyConstruct(spShape* shapeA, spShape* shapeB);

/// checks if a contact with a contact key exists
spBool spContactKeyExists(spContactKey key, spContact* contactList);

/// checks if two contact keys are the same
spBool spContactKeyEqual(spContactKey keyA, spContactKey keyB);

/// sorts the contact key so the first shape is < the second shape
void spContactKeySortShapes(spContactKey* key);

/// swap the shapes of a contact key
void spContactKeySwapShapes(spContactKey* key);

/// @{

#endif