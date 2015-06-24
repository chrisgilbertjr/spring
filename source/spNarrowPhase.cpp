
#include "spNarrowPhase.h"
#include "spContact.h"
#include "spBody.h"

spNarrowPhase
_spNarrowPhase(spContact* contact_list)
{
    return 
    { 
        spCollisionMatrix(), /// collision_matrix
        NULL        /// contact_list
    };
}

spBool 
spNarrowPhaseCollide(const spNarrowPhase* narrow, spContact* contact, const spCollisionInput& data)
{
    spCollisionMatrix matrix = narrow->collision_matrix;
    const spShapeType type_a = data.shape_a->type;
    const spShapeType type_b = data.shape_b->type;

    spCollisionFunc collision_func = spCollisionQueryFunc(matrix, type_a, type_b);
     return collision_func(contact, data);
}

void 
spNarrowPhaseStep(spNarrowPhase* narrow, spContact*& contact_list)
{
    spContact* contact = contact_list;
    while (contact != NULL)
    {
        spContactKey* key = &contact->key; ///< contact key of the contact
        spShape*      sa  = key->shape_a;  ///< shape a of the contact
        spShape*      sb  = key->shape_b;  ///< shape b of the contact
        spBody*       ba  = sa->body;      ///< body a of the contact
        spBody*       bb  = sb->body;      ///< body b of the contact
        spTransform*  xfa = &ba->xf;       ///< transform of body a
        spTransform*  xfb = &bb->xf;       ///< transform of body b

        const spCollisionInput data = spCollisionInput(sa, sb, xfa, xfb);

        /// collide the two shapes
        if (spNarrowPhaseCollide(narrow, contact, data) == spFalse)
        {
            /// destroy the contact
            spContact* destroy = contact;
            spContact* next = contact->next;
            spContactRemove(destroy, contact_list);
            spContactFree(destroy);
            contact = next;
        }
        else
        {
            contact = contact->next;
        }
    }
}
