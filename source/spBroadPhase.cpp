
#include "spBroadPhase.h"
#include "spShape.h"
#include "spBody.h"

spBroadPhase 
_spBroadPhase(spShape* shape_list)
{
    spBroadPhase broad;
    broad.contact_list = NULL;
    broad.shape_list = &shape_list;
    return broad;
}

void 
spBroadPhaseStep(spBroadPhase* broad, spBody* body_list, spContact*& contact_list)
{
    /// niave broadphase for now... O(n^2)
    for_each_body(body_a, body_list)
    {
        for_each_body(body_b, body_a->next)
        {
            for_each_shape(shape_a, body_a->shape_list)
            {
                for_each_shape(shape_b, body_b->shape_list)
                {
                    spBound* ba = &shape_a->bound; ///< bound of shape a
                    spBound* bb = &shape_b->bound; ///< bound of shape b

                    /// check if the shapes AABB's overlap
                    if (spBoundBoxOverlap(*ba, *bb, body_a->xf, body_b->xf) == spFalse) continue;

                    /// they do overlap, create a contact key
                    spContactKey key = spContactKey(shape_a, shape_b);

                    /// check if the contact key is currently in the contact list
                    if (spContactKeyExists(key, contact_list) == spFalse)
                    {
                        /// the contact key is not in the list, create a new contact with the key
                        spContact* contact = spContactNew(key);

                        /// insert the contact into the contact list
                        spContactAdd(contact, contact_list);
                    };
                }
            }
        }
    }
}