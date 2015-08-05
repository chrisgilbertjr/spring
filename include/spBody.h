
#ifndef SP_BODY_H
#define SP_BODY_H

#include "spLinkedList.h"
#include "spMath.h"

/// @defgroup spBody spBody
/// @{

/// different types of rigid body
enum spBodyType
{
    SP_BODY_KINEMATIC, ///< user controlled bodies
    SP_BODY_DYNAMIC,   ///< body that reacts to forces
    SP_BODY_STATIC,    ///< bodies that do no move. doesnt react to forces
};

/// a rigid body is an entity in the world. It describes where/how two things react when they collide.
/// you must attach collision shapes to rigid bodies for them to react to the world.
/// you can also attach constraints, which are restrictions on what a body can and cannot do.
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
    spWorld* world;            ///< world the body is in
    spLazyPointer userData;    ///< user data pointer
};

/// initialize a rigid body
void spBodyInit(spBody* body, spBodyType type);

/// frees the body and all shapes attached to the body
void spBodyDestroyShapes(spBody* body);

/// frees the body and all constraints attached to the body
void spBodyDestroyConstraints(spBody* body);

/// frees the body and all shapes/constraints attached to the body
void spBodyDestroy(spBody** bodyPtr);

/// allocates space for a rigid body on the heap
spBody* spBodyAlloc();

/// allocates space for a rigid body and initializes it with a type
spBody* spBodyNew(spBodyType type);

/// create a new kinematic body
spBody* spBodyNewKinematic();

/// create a new dynamic body
spBody* spBodyNewDynamic();

/// create a new static body
spBody* spBodyNewStatic();

/// frees the rigid body
void spBodyFree(spBody** body);

/// attach a shape to this body
void spBodyAddShape(spBody* body, spShape* shape);

/// remove a shape from this body
void spBodyRemoveShape(spBody* body, spShape* shape);

/// clear all forces acting on this body
void spBodyClearForces(spBody* body);

/// clear the bodies velocity
void spBodyClearVelocity(spBody* body);

/// integrates forces and updates velocity - semi-implicit euler
void spBodyIntegrateVelocity(spBody* body, const spVector& gravity, const spFloat h);

/// integrates velocity and updates position - semi-implicit euler
void spBodyIntegratePosition(spBody* body, const spFloat h);

/// calculates a bodies acceleration given gravity
spVector spBodyAcceleration(spBody* body, const spVector& gravity);

/// calculates the combined mass data of all shapes
void spBodyComputeShapeMassData(spBody* body);

/// get a local point in the world space of body
spVector spBodyLocalToWorldPoint(spBody* body, spVector point);

/// get a world point in the local space of body
spVector spBodyWorldToLocalPoint(spBody* body, spVector point);

/// get a local vector in the world space of body
spVector spBodyLocalToWorldVector(spBody* body, spVector vector);

/// get a world vector in the local space of body
spVector spBodyWorldToLocalVector(spBody* body, spVector vector);

/// apply a torque to the body
void spBodyApplyTorque(spBody* body, spFloat torque);

/// apply a force to the body from a point in local space
void spBodyApplyForceAtLocalPoint(spBody* body, spVector point, spVector force);

/// apply a force to the body from a point in world space
void spBodyApplyForceAtWorldPoint(spBody* body, spVector point, spVector force);

/// apply an impulse to the body at a point given an impulse
void spBodyApplyImpulseAtPoint(spBody* body, spVector point, spVector impulse);

/// apply an impulse to the body given the rel velocity to a point and an impulse
void spBodyApplyImpulse(spBody* body, spVector relVelocity, spVector impulse);

/// get the body's transform
spTransform spBodyGetTransform(spBody* body);

/// get the body's position
spVector spBodyGetPosition(spBody* body);

/// get the body's rotation
spRotation spBodyGetRotation(spBody* body);

/// get the body's angle
spFloat spBodyGetAngle(spBody* body);

/// get the body's center of mass
spVector spBodyGetCenterOfMass(spBody* body);

/// get the body's force
spVector spBodyGetForce(spBody* body);

/// get the body's gravity scale
spFloat spBodyGetGravityScale(spBody* body);

/// get the body's linear velocity damping
spFloat spBodyGetLinearVelocityDamping(spBody* body);

/// get the body's angular velocity damping
spFloat spBodyGetAngularVelocityDamping(spBody* body);

/// get the body's inertia
spFloat spBodyGetInertia(spBody* body);

/// get the body's mass
spFloat spBodyGetMass(spBody* body);

/// get the body's torque
spFloat spBodyGetTorque(spBody* body);

/// get the next body in the body list
spBody* spBodyGetNext(spBody* body);

/// get the prev body in the body list
spBody* spBodyGetPrev(spBody* body);

/// get the body's type
spBodyType spBodyGetType(spBody* body);

/// get the body's shape list
spShape* spBodyGetShapeList(spBody* body);

/// get the body's constraint list
spConstraint* spBodyGetConstraintList(spBody* body);

/// get the body's world
spWorld* spBodyGetWorld(spBody* body);

/// get the body's user data
spLazyPointer spBodyGetUserData(spBody* body);

/// sets the position and rotation of the rigid body
void spBodySetTransform(spBody* body, const spVector& position, spFloat angle);

/// sets the position of the rigid body
void spBodySetPosition(spBody* body, const spVector& position);

/// sets the rotation of the rigid body
void spBodySetRotation(spBody* body, spRotation rotate);

/// sets the rotation of the rigid body
void spBodySetAngle(spBody* body, spFloat angle);

/// sets the body's center of mass
void spBodySetCenterOfMass(spBody* body, spVector com);

/// sets the body's force, resets all other forces
void spBodySetForce(spBody* body, spVector force);

/// sets the body's gravity scale
void spBodySetGravityScale(spBody* body, spFloat gScale);

/// sets the body's linear velocity damping
void spBodySetLinearVelocityDamping(spBody* body, spFloat damping);

/// sets the body's angular velocity damping
void spBodySetAngularVelocityDamping(spBody* body, spFloat damping);

/// set the inertia of the body, overrides shape data.
/// @warning inertia reset when type is changed, or other shapes are added/removed
void spBodySetInertia(spBody* body, spFloat inertia);

/// set the mass of the body, overrides shape data.
/// @warning mass reset when type is changed, or other shapes are added/removed
void spBodySetMass(spBody* body, spFloat mass);

/// sets the body type, resets custom mass data to shapes mass data
void spBodySetType(spBody* body, spBodyType type);

/// sets the body's user data pointer
void spBodySetUserData(spBody* body, spLazyPointer* data);

/// @}

#endif