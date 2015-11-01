
#include "spContactKey.h"
#include "spContact.h"

void 
spContactKeyInit(spContactKey* key, spShape* shapeA, spShape* shapeB)
{
    NULLCHECK(key); NULLCHECK(shapeA); NULLCHECK(shapeB);
    if (spShapeLessThan(shapeA, shapeB))
    {
        key->shapeA = shapeA;
        key->shapeB = shapeB;
    }
    else
    {
        key->shapeA = shapeB;
        key->shapeB = shapeA;
    }
}

spContactKey 
spContactKeyConstruct(spShape* shapeA, spShape* shapeB)
{
    NULLCHECK(shapeA); NULLCHECK(shapeB);
    spContactKey key;
    spContactKeyInit(&key, shapeA, shapeB);
    return key;
}

spBool 
spContactKeyExists(spContactKey key, spContact* contactList)
{
    for (spContact* contact = contactList; contact != NULL; contact = contact->next)
    {
        if (spContactKeyEqual(key, contact->key) == spTrue)
        {
            return spTrue;
        }
    }
    return spFalse;
}

spBool 
spContactKeyEqual(spContactKey keyA, spContactKey keyB)
{
    return keyA.shapeA == keyB.shapeA && keyA.shapeB == keyB.shapeB ? spTrue : spFalse;
}

void 
spContactKeySortShapes(spContactKey* key)
{
    NULLCHECK(key);
    if (spShapeLessThan(key->shapeA, key->shapeB)) return;

    spContactKeySwapShapes(key);
}

void 
spContactKeySwapShapes(spContactKey* key)
{
    NULLCHECK(key);
    spShape* tmp = key->shapeA;
    key->shapeA = key->shapeB;
    key->shapeB = tmp;
}

