
#include "spConstraint.h"
#include "spWorld.h"
#include "spShape.h"
#include "spBody.h"

#ifdef SP_DEBUG
#define VALID(body) \
    NANCHECK(body->m);  NANCHECK(body->mInv); \
    NANCHECK(body->i);  NANCHECK(body->iInv); \
    NANCHECK(body->p.x); NANCHECK(body->p.y); \
    NANCHECK(body->v.x); NANCHECK(body->v.y); \
    NANCHECK(body->f.x); NANCHECK(body->f.y); \
    NANCHECK(body->a); \
    NANCHECK(body->w); \
    NANCHECK(body->t); \
    NULLCHECK(body);   \
    spAssert(body->m >= 0.0f, "mass is negative"); \
    spAssert(body->i >= 0.0f, "inertia is negative");
#else
    #define VALID(body) 
#endif

static void 
updateTransform(spBody* body)
{
    body->xf.q = spRotationConstruct(body->a);
    body->xf.p = spAdd(spMult(body->xf.q, body->com), body->p);
    VALID(body);
}

void 
spBodyInit(spBody* body, spBodyType type)
{
    NULLCHECK(body);
    body->xf = spTransformConstruct(spVectorZero(), spRotationZero());
    body->com = spVectorZero();
    body->p = spVectorZero();
    body->f = spVectorZero();
    body->v = spVectorZero();
    body->gScale = 1.0f;
    body->vDamp = 0.0f;
    body->wDamp = 0.0f;
    body->iInv = 0.0f;
    body->mInv = 0.0f;
    body->i = 0.0f;
    body->m = 0.0f;
    body->t = 0.0f;
    body->a = 0.0f;
    body->w = 0.0f;
    body->next = NULL;
    body->prev = NULL;
    body->i = 0.0f;
    body->m = 0.0f;
    body->world = NULL;
    body->shapes = NULL;
    body->userData = NULL;
    spBodySetType(body, type);
    VALID(body);
}

spBody* 
spBodyAlloc()
{
    return (spBody*)spMalloc(sizeof(spBody));
}

spBody* 
spBodyNew(spBodyType type)
{
    spBody* body = spBodyAlloc();
    spBodyInit(body, type);
    return body;
}

spBody* 
spBodyNewKinematic()
{
    return spBodyNew(SP_BODY_KINEMATIC);
}

spBody* 
spBodyNewDynamic()
{
    return spBodyNew(SP_BODY_DYNAMIC);
}

spBody* 
spBodyNewStatic()
{
    return spBodyNew(SP_BODY_STATIC);
}

void 
spBodyDestroyShapes(spBody* body)
{
    NULLCHECK(body);

    /// free any shapes that are attached to the body
    spShape* shape = body->shapes;
    while (shape)
    {
        spShape* next = shape->next;
        spShapeFree(shape);
        shape = next;
    }
}

void 
spBodyDestroyConstraints(spBody* body)
{
    NULLCHECK(body);

    /// free any constraints this body is involved with
    spConstraint* constraint = body->world->jointList;
    while (constraint)
    {
        spConstraint* next = constraint->next;
        if (constraint->bodyA == body || constraint->bodyB == body)
        {
            spConstraintFree(constraint);
        }
        constraint = next;
    }
}

void 
spBodyDestroy(spBody** bodyPtr)
{
    NULLCHECK(bodyPtr);
    spBody* body = *bodyPtr;

    /// destroy shapes/constraints
    spBodyDestroyShapes(body);
    spBodyDestroyConstraints(body);

    /// free the memory
    spBodyFree(bodyPtr);
}

void 
spBodyFree(spBody** bodyPtr)
{
    spBody* body = *bodyPtr;
    NULLCHECK(body);

    if (body->world)
    {
        spWorldRemoveBody(body->world, body);
    }
    spFree(bodyPtr);
}

void 
spBodyAddShape(spBody* body, spShape* shape)
{
    SP_LINKED_LIST_PREPEND(spShape, shape, body->shapes);
    if (body->type == SP_BODY_DYNAMIC)
    {
        spBodyComputeShapeMassData(body);
    }
    shape->body = body;
}

void 
spBodyRemoveShape(spBody* body, spShape* shape)
{
    SP_LINKED_LIST_REMOVE(spShape, shape, body->shapes);
    if (body->type == SP_BODY_DYNAMIC && body->shapes != NULL)
    {
        spBodyComputeShapeMassData(body);
    }
}

void 
spBodyClearForces(spBody* body)
{
    body->f = spVectorZero();
    body->t = 0.0f;
}

void 
spBodyIntegrateVelocity(spBody* body, const spVector& gravity, const spFloat h)
{
    /// v += dt * acceleration
    /// w += dt * inv_mass * torque
    /// v *= 1.0 / 1.0 + dt + v_damping
    /// w *= 1.0 / 1.0 + dt + w_damping

    if (body->type != SP_BODY_DYNAMIC) return;

    /// integrate forces to get new velocities
    body->v  = spAdd(body->v, spMult(h, spBodyAcceleration(body, gravity)));
    body->w += h * body->mInv * body->t;

    /// apply damping
    body->v  = spMult(body->v, 1.0f / (1.0f + h * body->vDamp));
    body->w *= 1.0f / (1.0f + h * body->wDamp);

    spBodyClearForces(body);
}

void 
spBodyIntegratePosition(spBody* body, const spFloat h)
{
    /// position += h * velocity
    /// rotation += h * angular_velocity

    /// integrate velocity to get new position/rotation
    body->p = spAdd(body->p, spMult(h, body->v));
    body->a = body->a + body->w * h;

    /// update the bodys' transform according to the new position/rotation
    updateTransform(body);
}

spVector 
spBodyAcceleration(spBody* body, const spVector& gravity)
{
    /// F = ma
    /// => a = F(1/m)
    /// a = gravity + F(1/m)
    return spAdd(spMult(body->gScale, gravity), spMult(body->f, body->mInv));
}

void spBodyComputeShapeMassData(spBody* body)
{
    spAssert(body != NULL, "body is null while computing its mass data");
    spAssert(body->type == SP_BODY_DYNAMIC, "body type is not dynamic while computing 5its mass data");
    
    /// reset the bodys' mass data
    body->com = spVectorZero();
    body->i = 0.0f;
    body->m = 0.0f;

    spVector position = body->xf.p;

    spFloat tmass, t0mass;
    spFloat tinertia;
    spVector tcom;
    tinertia = tmass = t0mass = 0.0f;
    tcom = spVectorZero();

    /// accumulate mass data
    for (spShape* shape = body->shapes; shape; shape = shape->next)
    {
        spMassData* data = &shape->mass_data;
        spFloat m = data->mass;
        spFloat i = data->inertia;
        spVector com = data->com;

        if (m > 0.0f)
        {
            t0mass = tmass;
            tmass += m;
            spFloat tmass_inv = 1.0f / tmass;
            tinertia += m * i + spDistanceSquared(tcom, com) * m * t0mass * tmass_inv;
            tcom = spLerp(tcom, com, m * tmass_inv);
        }
    }

    body->mInv = tmass ? 1.0f / tmass : 0.0f;
    body->m    = tmass ? tmass : SP_INFINITY;
    body->iInv = tinertia ? 1.0f / tinertia : 0.0f;
    body->i    = tinertia ? tinertia : SP_INFINITY;
    body->com  = tcom;

    spBodySetPosition(body, position);
    VALID(body);
}

spVector
spBodyLocalToWorldPoint(spBody* body, spVector point)
{
    return spMult(body->xf, point);
}

spVector
spBodyWorldToLocalPoint(spBody* body, spVector point)
{
    return spTMult(body->xf, point);
}

spVector 
spBodyLocalToWorldVector(spBody* body, spVector vector)
{
    return spMult(body->xf.q, vector);
}

spVector 
spBodyWorldToLocalVector(spBody* body, spVector vector)
{
    return spTMult(body->xf.q, vector);
}

void 
spBodyApplyTorque(spBody* body, spFloat torque)
{
    body->t += torque;
}

void 
spBodyApplyForceAtLocalPoint(spBody* body, spVector point, spVector force)
{
    spBodyApplyForceAtWorldPoint(body, spMult(body->xf, point), spMult(body->xf.q, force));
}

void 
spBodyApplyForceAtWorldPoint(spBody* body, spVector point, spVector force)
{
    body->f  = spAdd(body->f, force);
    body->t += spCross(spSub(point, spMult(body->xf, body->com)), force);
}

void 
spBodyApplyImpulseAtPoint(spBody* body, spVector point, spVector impulse)
{
    spBodyApplyImpulse(body, impulse, spSub(point, body->p));
}

void 
spBodyApplyImpulse(spBody* body, spVector relVelocity, spVector impulse)
{
    body->v  = spAdd(body->v, spMult(impulse, body->mInv));
    body->w += body->iInv * spCross(relVelocity, impulse);
}

spTransform 
spBodyGetTransform(spBody* body)
{
    return body->xf;
}

spVector 
spBodyGetPosition(spBody* body)
{
    return body->xf.p;
}

spRotation 
spBodyGetRotation(spBody* body)
{
    return body->xf.q;
}

spFloat 
spBodyGetAngle(spBody* body)
{
    return spRotationGetAngleDeg(body->xf.q);
}

spVector 
spBodyGetCenterOfMass(spBody* body)
{
    return body->com;
}

spVector 
spBodyGetForce(spBody* body)
{
    return body->f;
}

spFloat 
spBodyGetGravityScale(spBody* body)
{
    return body->gScale;
}

spFloat 
spBodyGetLinearVelocityDamping(spBody* body)
{
    return body->vDamp;
}

spFloat 
spBodyGetAngularVelocityDamping(spBody* body)
{
    return body->wDamp;
}

spFloat 
spBodyGetInertia(spBody* body)
{
    return body->i;
}

spFloat 
spBodyGetMass(spBody* body)
{
    return body->m;
}

spFloat 
spBodyGetTorque(spBody* body)
{
    return body->t;
}

spBody* 
spBodyGetNext(spBody* body)
{
    return body->next;
}

spBody* 
spBodyGetPrev(spBody* body)
{
    return body->prev;
}

spBodyType 
spBodyGetType(spBody* body)
{
    return body->type;
}

spShape* 
spBodyGetShapeList(spBody* body)
{
    return body->shapes;
}

spWorld* 
spBodyGetWorld(spBody* body)
{
    return body->world;
}

spLazyPointer 
spBodyGetUserData(spBody* body)
{
    return body->userData;
}

void 
spBodySetTransform(spBody* body, const spVector& position, spFloat angle)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    body->a = angle * SP_DEG_TO_RAD;
    updateTransform(body);
}

void 
spBodySetPosition(spBody* body, const spVector& position)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    updateTransform(body);
}

void 
spBodySetRotation(spBody* body, spRotation rotate)
{
    body->a = spRotationGetAngle(rotate);
    updateTransform(body);
}

void 
spBodySetAngle(spBody* body, spFloat angle)
{
    body->a = angle * SP_DEG_TO_RAD;
    updateTransform(body);
}

void 
spBodySetCenterOfMass(spBody* body, spVector com)
{
    body->com = com;
}

void 
spBodySetForce(spBody* body, spVector force)
{
    body->f = force;
}

void 
spBodySetGravityScale(spBody* body, spFloat gScale)
{
    body->gScale = gScale;
}

void 
spBodySetLinearVelocityDamping(spBody* body, spFloat damping)
{
    body->vDamp = damping;
}

void 
spBodySetAngularVelocityDamping(spBody* body, spFloat damping)
{
    body->wDamp = damping;
}

void
spBodySetInertia(spBody* body, spFloat inertia)
{
    spAssert(inertia > 0.0f, "inertia must be positive!");

    body->i = inertia;
    body->iInv = 1.0f / inertia;
    VALID(body);
}

void
spBodySetMass(spBody* body, spFloat mass)
{
    spAssert(body->type == SP_BODY_DYNAMIC, "The body is not dynamic, cannot set mass!");
    spAssert(mass > 0.0f, "mass must be positive!");

    body->m = mass;
    body->mInv = 1.0f / mass;
    VALID(body);
}

void 
spBodySetType(spBody* body, spBodyType type)
{
    body->type = type;

    switch (type)
    {
    case SP_BODY_DYNAMIC:
        spBodyComputeShapeMassData(body);
        break;
    case SP_BODY_KINEMATIC:
    case SP_BODY_STATIC:
        body->w = body->mInv = body->iInv = 0.0f;
        body->m = body->i = SP_INFINITY;
        body->v = spVectorZero();
    }
}

void 
spBodySetUserData(spBody* body, spLazyPointer* data)
{
    body->userData = data;
}