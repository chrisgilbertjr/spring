
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
SPRING_API void spContactKeyInit(spContactKey* key, spShape* shapeA, spShape* shapeB);

/// 'faked' constructor for stack allocation
SPRING_API spContactKey spContactKeyConstruct(spShape* shapeA, spShape* shapeB);

/// checks if a contact with a contact key exists
SPRING_API spBool spContactKeyExists(spContactKey key, spContact* contactList);

/// checks if two contact keys are the same
SPRING_API spBool spContactKeyEqual(spContactKey keyA, spContactKey keyB);

/// sorts the contact key so the first shape is < the second shape
SPRING_API void spContactKeySortShapes(spContactKey* key);

/// swap the shapes of a contact key
SPRING_API void spContactKeySwapShapes(spContactKey* key);

/// @{

#endif