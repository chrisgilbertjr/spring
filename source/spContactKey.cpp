
#include "spContactKey.h"
#include "spContact.h"

void 
spContactKeyInit(spContactKey* key, spShape* shape_a, spShape* shape_b)
{
    if (spShapeLessThan(shape_a, shape_b))
    {
        key->shape_a = shape_a;
        key->shape_b = shape_b;
    }
    else
    {
        key->shape_a = shape_b;
        key->shape_b = shape_a;
    }
}

spContactKey 
_spContactKey(spShape* shape_a, spShape* shape_b)
{
    spContactKey key;
    spContactKeyInit(&key, shape_a, shape_b);
    return key;
}

spBool 
spContactKeyExists(const spContactKey& key, spContact* contact_list)
{
    for_each_contact(contact, contact_list)
    {
        if (spContactKeyEqual(key, contact->key) == spTrue)
        {
            return spTrue;
        }
    }
    return spFalse;
}

spBool 
spContactKeyEqual(const spContactKey& key_a, const spContactKey& key_b)
{
    return key_a.shape_a == key_b.shape_a && key_a.shape_b == key_b.shape_b ? spTrue : spFalse;
}

void 
spContactKeySortShapes(spContactKey* key)
{
    if (spShapeLessThan(key->shape_a, key->shape_b)) return;

    spContactKeySwapShapes(key);
}

void 
spContactKeySwapShapes(spContactKey* key)
{
    spShape* tmp = key->shape_a;
    key->shape_a = key->shape_b;
    key->shape_b = tmp;
}

