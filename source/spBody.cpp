
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
    body->type = type;
    body->shape_list = NULL;
    body->constraint_list = NULL;
    body->user_data = NULL;
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
spBodyRemove(spBody* body, spBody*& body_list)
{
    SP_LINKED_LIST_REMOVE(spBody, body, body_list);
}

void 
spBodySetPosition(spBody* body, const spVector& position)
{
    body->p = spAdd(spMult(body->xf.q, body->com), position);
    spBodyIsSane(body);
    spBodySetTransform(body, body->p, body->a);
}

void 
spBodySetRotation(spBody* body, spFloat angle)
{
    body->a = angle;
    spBodyIsSane(body);
    spBodySetTransform(body, body->p, body->a);
}

void 
spBodySetTransform(spBody* body, const spVector& p, const spFloat a)
{
    /// new_position = rotate(center_of_mass, angle) + position
    body->xf.q = spRotation(a);
    body->xf.p = spAdd(spMult(body->xf.q, body->com), p);
    spBodyIsSane(body);
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
    spBodySetTransform(body, body->p, body->a);

    /// reset psuedo velocities if we use them
    /// spBodyClearPsuedoVelocities(body);
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

    /// accumulate mass data, reduce calls to body struct for better cache performance
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

    body->m_inv = 1.0f / tmass;
    body->m = tmass;
    body->i_inv = 1.0f / tinertia;
    body->i = tinertia;
    body->com = tcom;

    spBodySetPosition(body, position);
    spBodyIsSane(body);
}