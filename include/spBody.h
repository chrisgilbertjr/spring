
#ifndef SP_BODY_H
#define SP_BODY_H

#include "spLinkedList.h"
#include "spMath.h"

/// @defgroup spBody spBody
/// @{

enum spBodyType
{
    SP_BODY_KINEMATIC,
    SP_BODY_DYNAMIC,
    SP_BODY_STATIC,
};

struct spBody
{
    spTransform xf;                ///< rigid body transform 
    spVector com;                  ///< center of mass in local 
    spVector p;                    ///< position of the com in world space
    spVector f;                    ///< force to be applied to the body during integration
    spVector v;                    ///< linear velocity
    //spVector v_bias;               ///<
    //spFloat w_bias;                ///<
    spFloat g_scale;               ///< gravity scale on this rigid body
    spFloat v_damp;                ///< linear velocity dampening applied during integration
    spFloat w_damp;                ///< angular velocity dampening applied during integration
    spFloat i_inv;                 ///< inverse inertia
    spFloat m_inv;                 ///< inverse mass
    spFloat i;                     ///< inertia
    spFloat m;                     ///< mass
    spFloat t;                     ///< torque to be applied to the body during integration
    spFloat a;                     ///< rotation angle around com in world space
    spFloat w;                     ///< angular velocity
    spBody* next;                  ///< the next body in the linked list of bodies
    spBody* prev;                  ///< the previous body in the linked list of bodies
    spBool can_sleep;              ///< can this rigid body sleep? (bypass simulation when static)
    spBodyType type;               ///< the type of the body, which describes how it is simulated
    spShape* shape_list;           ///<
    spConstraint* constraint_list; ///<
    spLazyPointer user_data;       ///< user data pointer
};

/// initialize a rigid body
void spBodyInit(spBody* body, spBodyType type);

/// allocates space for a rigid body on the heap
spBody* spBodyAlloc();

/// allocates space for a rigid body and initializes it with a type
spBody* spBodyNew(spBodyType type);

/// frees the rigid body
void spBodyFree(spBody*& body);

/// add a body into the body list
void spBodyAdd(spBody* body, spBody*& body_list);

/// remove a body from the body list
void spBodyRemove(spBody* body, spBody*& body_list);

/// sets the position of the rigid body
void spBodySetPosition(spBody* body, const spVector& position);

/// sets the rotation of the rigid body
void spBodySetRotation(spBody* body, spFloat angle);

/// set the bodys' transform according to a new position and rotation angle
void spBodySetTransform(spBody* body, const spVector& p, const spFloat a);

/// set the mass and update mass properties of a rigid body
void spBodySetMass(spBody* body, spFloat mass);

/// set the inertia and update inertia properties of a rigid body
void spBodySetInertia(spBody* body, spFloat inertia);

/// clear all forces acting on this body
void spBodyClearForces(spBody* body);

/// integrates forces and updates velocity - semi-implicit euler
void spBodyIntegrateVelocity(spBody* body, const spVector& gravity, const spFloat h);

/// integrates velocity and updates position - semi-implicit euler
void spBodyIntegratePosition(spBody* body, const spFloat h);

/// calculates a bodies acceleration given gravity
spVector spBodyAcceleration(spBody* body, const spVector& gravity);

/// calculates the combined mass data of all shapes
void spBodyComputeShapeMassData(spBody* body);

/// sanity check
#ifdef SP_DEBUG
 #define spBodyIsSane(body) _spBodyIsSane(body)
 inline void _spBodyIsSane(spBody* body)
 {
     spAssert(body->m == body->m && body->m_inv == body->m_inv, "mass is NaN in sanity check");
     spAssert(body->i == body->i && body->i_inv == body->i_inv, "inertia is NaN in sanity check");
     spAssert(body->p.x == body->p.x && body->p.y == body->p.y, "position contains NaN in sanity check");
     spAssert(body->v.x == body->v.x && body->v.y == body->v.y, "velocity contains NaN in sanity check");
     spAssert(body->f.x == body->f.x && body->f.y == body->f.y, "force contains NaN in sanity check");
     spAssert(body->m >= 0.0f, "mass is negative in sanity check");
     spAssert(body->i >= 0.0f, "inertia is negative in sanity check");
     spAssert(body->a == body->a, "angle is NaN in sanity check");
     spAssert(body->w == body->w, "angular velocity is NaN in sanity check");
     spAssert(body->t == body->t, "torque is Nan in sanity check");
 }
#else
 #define spBodyIsSane(body) 
#endif

/// @}

#endif