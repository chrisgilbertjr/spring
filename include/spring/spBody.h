
#ifndef SP_BODY_H
#define SP_BODY_H

#include "spLinkedList.h"
#include "spMath.h"

/// @defgroup spBody spBody
/// @{

/// different types of rigid body
typedef enum 
{
    SP_BODY_KINEMATIC, ///< user controlled bodies
    SP_BODY_DYNAMIC,   ///< body that reacts to forces
    SP_BODY_STATIC,    ///< bodies that do no move. doesnt react to forces
} spBodyType;

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
SPRING_API void spBodyInit(spBody* body, spBodyType type);

/// frees the body and all shapes attached to the body
SPRING_API void spBodyDestroyShapes(spBody* body);

/// frees the body and all constraints attached to the body
SPRING_API void spBodyDestroyConstraints(spBody* body);

/// frees the body and all shapes/constraints attached to the body
SPRING_API void spBodyDestroy(spBody** bodyPtr);

/// allocates space for a rigid body on the heap
SPRING_API spBody* spBodyAlloc();

/// allocates space for a rigid body and initializes it with a type
SPRING_API spBody* spBodyNew(spBodyType type);

/// create a new kinematic body
SPRING_API spBody* spBodyNewKinematic();

/// create a new dynamic body
SPRING_API spBody* spBodyNewDynamic();

/// create a new static body
SPRING_API spBody* spBodyNewStatic();

/// frees the rigid body
SPRING_API void spBodyFree(spBody** body);

/// attach a shape to this body
SPRING_API void spBodyAddShape(spBody* body, spShape* shape);

/// remove a shape from this body
SPRING_API void spBodyRemoveShape(spBody* body, spShape* shape);

/// clear all forces acting on this body
SPRING_API void spBodyClearForces(spBody* body);

/// clear the bodies velocity
SPRING_API void spBodyClearVelocity(spBody* body);

/// integrates forces and updates velocity - semi-implicit euler
SPRING_API void spBodyIntegrateVelocity(spBody* body, const spVector gravity, const spFloat h);

/// integrates velocity and updates position - semi-implicit euler
SPRING_API void spBodyIntegratePosition(spBody* body, const spFloat h);

/// calculates a bodies acceleration given gravity
SPRING_API spVector spBodyAcceleration(spBody* body, const spVector gravity);

/// calculates the combined mass data of all shapes
SPRING_API void spBodyComputeShapeMassData(spBody* body);

/// get a local point in the world space of body
SPRING_API spVector spBodyLocalToWorldPoint(spBody* body, spVector point);

/// get a world point in the local space of body
SPRING_API spVector spBodyWorldToLocalPoint(spBody* body, spVector point);

/// get a local vector in the world space of body
SPRING_API spVector spBodyLocalToWorldVector(spBody* body, spVector vector);

/// get a world vector in the local space of body
SPRING_API spVector spBodyWorldToLocalVector(spBody* body, spVector vector);

/// apply a torque to the body
SPRING_API void spBodyApplyTorque(spBody* body, spFloat torque);

/// apply a force to the body from a point in local space
SPRING_API void spBodyApplyForceAtLocalPoint(spBody* body, spVector point, spVector force);

/// apply a force to the body from a point in world space
SPRING_API void spBodyApplyForceAtWorldPoint(spBody* body, spVector point, spVector force);

/// apply an impulse to the body at a point given an impulse
SPRING_API void spBodyApplyImpulseAtPoint(spBody* body, spVector point, spVector impulse);

/// apply an impulse to the body given the rel velocity to a point and an impulse
SPRING_API void spBodyApplyImpulse(spBody* body, spVector relVelocity, spVector impulse);

/// get the body's transform
SPRING_API spTransform spBodyGetTransform(spBody* body);

/// get the body's position
SPRING_API spVector spBodyGetPosition(spBody* body);

/// get the body's rotation
SPRING_API spRotation spBodyGetRotation(spBody* body);

/// get the body's angle
SPRING_API spFloat spBodyGetAngle(spBody* body);

/// get the body's center of mass
SPRING_API spVector spBodyGetCenterOfMass(spBody* body);

/// get the body's force
SPRING_API spVector spBodyGetForce(spBody* body);

/// get the body's gravity scale
SPRING_API spFloat spBodyGetGravityScale(spBody* body);

/// get the body's linear velocity damping
SPRING_API spFloat spBodyGetLinearVelocityDamping(spBody* body);

/// get the body's angular velocity damping
SPRING_API spFloat spBodyGetAngularVelocityDamping(spBody* body);

/// get the body's inertia
SPRING_API spFloat spBodyGetInertia(spBody* body);

/// get the body's mass
SPRING_API spFloat spBodyGetMass(spBody* body);

/// get the body's torque
SPRING_API spFloat spBodyGetTorque(spBody* body);

/// get the next body in the body list
SPRING_API spBody* spBodyGetNext(spBody* body);

/// get the prev body in the body list
SPRING_API spBody* spBodyGetPrev(spBody* body);

/// get the body's type
SPRING_API spBodyType spBodyGetType(spBody* body);

/// get the body's shape list
SPRING_API spShape* spBodyGetShapeList(spBody* body);

/// get the body's constraint list
SPRING_API spConstraint* spBodyGetConstraintList(spBody* body);

/// get the body's world
SPRING_API spWorld* spBodyGetWorld(spBody* body);

/// get the body's user data
SPRING_API spLazyPointer spBodyGetUserData(spBody* body);

/// sets the position and rotation of the rigid body
SPRING_API void spBodySetTransform(spBody* body, const spVector position, spFloat angle);

/// sets the position of the rigid body
SPRING_API void spBodySetPosition(spBody* body, const spVector position);

/// sets the rotation of the rigid body
SPRING_API void spBodySetRotation(spBody* body, spRotation rotate);

/// sets the rotation of the rigid body
SPRING_API void spBodySetAngle(spBody* body, spFloat angle);

/// sets the body's center of mass
SPRING_API void spBodySetCenterOfMass(spBody* body, spVector com);

/// sets the body's force, resets all other forces
SPRING_API void spBodySetForce(spBody* body, spVector force);

/// sets the body's gravity scale
SPRING_API void spBodySetGravityScale(spBody* body, spFloat gScale);

/// sets the body's linear velocity damping
SPRING_API void spBodySetLinearVelocityDamping(spBody* body, spFloat damping);

/// sets the body's angular velocity damping
SPRING_API void spBodySetAngularVelocityDamping(spBody* body, spFloat damping);

/// set the inertia of the body, overrides shape data.
/// @warning inertia reset when type is changed, or other shapes are added/removed
SPRING_API void spBodySetInertia(spBody* body, spFloat inertia);

/// set the mass of the body, overrides shape data.
/// @warning mass reset when type is changed, or other shapes are added/removed
SPRING_API void spBodySetMass(spBody* body, spFloat mass);

/// sets the body type, resets custom mass data to shapes mass data
SPRING_API void spBodySetType(spBody* body, spBodyType type);

/// sets the body's user data pointer
SPRING_API void spBodySetUserData(spBody* body, spLazyPointer* data);

/// @}

#endif