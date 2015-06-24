
#include "spShape.h"
#include "spBody.h"

void 
spBodyInit(spBody* body, spBodyType type)
{
    body->xf = spTransform(spVectorZero(), spRotationZero());
    body->com = spVectorZero();
    body->p = spVectorZero();
    body->f = spVectorZero();
    body->v = spVectorZero();
    body->g_scale = 1.0f;
    body->v_damp = 0.0f;
    body->w_damp = 0.0f;
    body->i_inv = 0.0f;
    body->m_inv = 0.0f;
    body->i = 0.0f;
    body->m = 0.0f;
    body->t = 0.0f;
    body->a = 0.0f;
    body->w = 0.0f;
    body->next = NULL;
    body->prev = NULL;
    body->can_sleep = spTrue;
    body->i = 0.0f;
    body->m = 0.0f;
    body->shape_list = NULL;
    body->constraint_list = NULL;
    body->user_data = NULL;
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
spBodyFree(spBody*& body)
{
    spFree(body);
}

void 
spBodyAdd(spBody* body, spBody*& body_list)
{
    SP_LINKED_LIST_PREPEND(spBody, body, body_list);
}

void 
spBodyAddShape(spBody* body, spShape* shape)
{
    SP_LINKED_LIST_PREPEND(spShape, shape, body->shape_list);
    if (body->type == SP_BODY_DYNAMIC)
    {
        spBodyComputeShapeMassData(body);
    }
}

void 
spBodyRemove(spBody* body, spBody*& body_list)
{
    SP_LINKED_LIST_REMOVE(spBody, body, body_list);
}

void 
spBodySetPosition(spBody* body, const spVector& position)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    __spBodyUpdateTransform(body);
}

void 
spBodySetRotation(spBody* body, spFloat angle)
{
    body->a = angle * SP_DEG_TO_RAD;
    __spBodyUpdateTransform(body);
}

void 
spBodySetTransform(spBody* body, const spVector& position, spFloat angle)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    body->a = angle * SP_DEG_TO_RAD;
    __spBodyUpdateTransform(body);
}

void 
__spBodyUpdateTransform(spBody* body)
{
    /// new_position = rotate(center_of_mass, angle) + position
    body->xf.q = spRotation(body->a);
    body->xf.p = spAdd(spMult(body->xf.q, body->com), body->p);
    spBodyIsSane(body);
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
        body->w = body->m_inv = body->i_inv = 0.0f;
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
    body->m_inv = 1.0f / mass;
    spBodyIsSane(body);
}

void 
spBodySetInertia(spBody* body, spFloat inertia)
{
    spAssert(inertia > 0.0f, "inertia must be positive!");

    body->i = inertia;
    body->i_inv = 1.0f / inertia;
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
    body->w += h * body->m_inv * body->t;

    /// apply damping
    body->v  = spMult(body->v, 1.0f / (1.0f + h * body->v_damp));
    body->w *= 1.0f / (1.0f + h * body->w_damp);

    spBodyClearForces(body);
}

void 
spBodyIntegratePosition(spBody* body, const spFloat h)
{
    /// position += h * velocity
    /// rotation += h * angular_velocity

    /// integrate velocity to get new position/rotation
    body->p = spAdd(body->p, spMult(h, body->v));
    body->a = body->a + h * body->w;

    /// update the bodys' transform according to the new position/rotation
    __spBodyUpdateTransform(body);

    /// reset psuedo velocities if we use them
    /// spBodyClearPsuedoVelocities(body);
}

 void _spBodyIsSane(spBody* body)
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

spVector 
spBodyAcceleration(spBody* body, const spVector& gravity)
{
    /// F = ma
    /// => a = F(1/m)
    /// a = gravity + F(1/m)
    return spAdd(spMult(body->g_scale, gravity), spMult(body->f, body->m_inv));
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
    for_each_shape(shape, body->shape_list)
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

    body->m_inv = tmass ? 1.0f / tmass : 0.0f;
    body->m = tmass ? tmass : SP_INFINITY;
    body->i_inv = tinertia ? 1.0f / tinertia : 0.0f;
    body->i = tinertia ? tinertia : SP_INFINITY;
    body->com = tcom;

    spBodySetPosition(body, position);
    spBodyIsSane(body);
}

void 
spBodyApplyTorque(spBody* body, spFloat torque)
{
    body->t += torque;
}