
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
    spTransform xf;            ///< rigid body transform 
    spVector com;              ///< center of mass in local 
    spVector p;                ///< position of the com in world space
    spVector f;                ///< force to be applied to the body during integration
    spVector v;                ///< linear velocity
    spFloat gScale;            ///< gravity scale on this rigid body
    spFloat vDamp;             ///< linear velocity dampening applied during integration
    spFloat wDamp;             ///< angular velocity dampening applied during integration
    spFloat iInv;              ///< inverse inertia
    spFloat mInv;              ///< inverse mass
    spFloat i;                 ///< inertia
    spFloat m;                 ///< mass
    spFloat t;                 ///< torque to be applied to the body during integration
    spFloat a;                 ///< rotation angle around com in world space
    spFloat w;                 ///< angular velocity
    spBody* next;              ///< the next body in the linked list of bodies
    spBody* prev;              ///< the previous body in the linked list of bodies
    spBodyType type;           ///< the type of the body, which describes how it is simulated
    spShape* shapes;           ///< shapes attached to a body
    spConstraint* constraints; ///< constraints affecting the body
    spLazyPointer userData;    ///< user data pointer
};

/// initialize a rigid body
void spBodyInit(spBody* body, spBodyType type);

/// allocates space for a rigid body on the heap
spBody* spBodyAlloc();

/// allocates space for a rigid body and initializes it with a type
spBody* spBodyNew(spBodyType type);

/// TODO:
spBody* spBodyNewKinematic();

/// TODO:
spBody* spBodyNewDynamic();

/// TODO:
spBody* spBodyNewStatic();

/// frees the rigid body
void spBodyFree(spBody** body);

/// add a body into the body list
/// DEPRECATED:
void spBodyAdd(spBody* body, spBody*& body_list);

/// TODO:
void spBodyAddShape(spBody* body, spShape* shape);

/// remove a body from the body list
void spBodyRemove(spBody* body, spBody*& body_list);

/// sets the position of the rigid body
void spBodySetPosition(spBody* body, const spVector& position);

/// sets the rotation of the rigid body
void spBodySetRotation(spBody* body, spFloat angle);

/// sets the position and rotation of the rigid body
void spBodySetTransform(spBody* body, const spVector& position, spFloat angle);

/// TODO:
void spBodySetType(spBody* body, spBodyType type);

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

/// TODO:
void spBodyApplyTorque(spBody* body, spFloat torque);

/// TODO:
void spBodyApplyImpulseAtPoint(spBody* body, spVector point, spVector impulse);

/// TODO:
void spBodyApplyImpulse(spBody* body, spVector relVelocity, spVector impulse);

/// sanity check
#ifdef SP_DEBUG
 #define spBodyIsSane(body) _spBodyIsSane(body)
 void _spBodyIsSane(spBody* body);
#else
 #define spBodyIsSane(body) 
#endif

/// @}

#endif