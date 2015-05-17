
#ifndef SP_CONTACT_KEY_H
#define SP_CONTACT_KEY_H

#include "spShape.h"

/// @defgroup spContactKey spContactKey
/// @{

struct spContactKey
{
    spShape* shape_a;
    spShape* shape_b;
};

/// initialize a contact key with two shapes
void spContactKeyInit(spContactKey* key, spShape* shape_a, spShape* shape_b);

/// 'faked' constructor for stack allocation
spContactKey _spContactKey(spShape* shape_a, spShape* shape_b);

/// checks if a contact with a contact key exists
spBool spContactKeyExists(const spContactKey& key, spContact* contact_list);

/// checks if two contact keys are the same
spBool spContactKeyEqual(const spContactKey& key_a, const spContactKey& key_b);

/// sorts the contact key so the first shape is < the second shape
void spContactKeySortShapes(spContactKey* key);

/// swap the shapes of a contact key
void spContactKeySwapShapes(spContactKey* key);

/// @{

#endif