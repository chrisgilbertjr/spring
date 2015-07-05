
#include "spShape.h"
#include "spBody.h"

static void 
updateTransform(spBody* body)
{
    /// new_position = rotate(center_of_mass, angle) + position
    body->xf.q = spRotation(body->a);
    body->xf.p = spAdd(spMult(body->xf.q, body->com), body->p);
    spBodyIsSane(body);
}

void 
spBodyInit(spBody* body, spBodyType type)
{
    body->xf = spTransform(spVectorZero(), spRotationZero());
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
    body->shapes = NULL;
    body->constraints = NULL;
    body->userData = NULL;
    spBodySetType(body, type);
    spBodyIsSane(body);
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
spBodyFree(spBody** body)
{
    spFree(body);
}

void 
spBodyAddShape(spBody* body, spShape* shape)
{
    SP_LINKED_LIST_PREPEND(spShape, shape, body->shapes);
    if (body->type == SP_BODY_DYNAMIC)
    {
        spBodyComputeShapeMassData(body);
    }
}

void 
spBodySetPosition(spBody* body, const spVector& position)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    updateTransform(body);
}

void 
spBodySetRotation(spBody* body, spFloat angle)
{
    body->a = angle * SP_DEG_TO_RAD;
    updateTransform(body);
}

void 
spBodySetTransform(spBody* body, const spVector& position, spFloat angle)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    body->a = angle * SP_DEG_TO_RAD;
    updateTransform(body);
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
spBodySetMass(spBody* body, spFloat mass)
{
    spAssert(body->type == SP_BODY_DYNAMIC, "The body is not dynamic, cannot set mass!");
    spAssert(mass > 0.0f, "mass must be positive!");

    body->m = mass;
    body->mInv = 1.0f / mass;
    spBodyIsSane(body);
}

void 
spBodySetInertia(spBody* body, spFloat inertia)
{
    spAssert(inertia > 0.0f, "inertia must be positive!");

    body->i = inertia;
    body->iInv = 1.0f / inertia;
    spBodyIsSane(body);
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

 void _spBodyIsSane(spBody* body)
 {
     spAssert(body->m == body->m && body->mInv == body->mInv, "mass is NaN in sanity check");
     spAssert(body->i == body->i && body->iInv == body->iInv, "inertia is NaN in sanity check");
     spAssert(body->p.x == body->p.x && body->p.y == body->p.y, "position contains NaN in sanity check");
     spAssert(body->v.x == body->v.x && body->v.y == body->v.y, "velocity contains NaN in sanity check");
     spAssert(body->f.x == body->f.x && body->f.y == body->f.y, "force contains NaN in sanity check");
     spAssert(body->m >= 0.0f, "mass is negative in sanity check");
     spAssert(body->i >= 0.0f, "inertia is negative in sanity check");
     spAssert(body->a == body->a, "angle is NaN in sanity check");
     spAssert(body->w == body->w, "angular velocity is NaN in sanity check");
     spAssert(body->t == body->t, "torque is Nan in sanity check");
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
    spAssert(body->type == SP_BODY_DYNAMIC, "body type is not dynamic while computing its mass data");
    
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
    for_each_shape(shape, body->shapes)
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
    spBodyIsSane(body);
}

void 
spBodyApplyTorque(spBody* body, spFloat torque)
{
    body->t += torque;
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