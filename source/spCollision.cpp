
#include "spCollision.h"
#include "spContact.h"
#include "spDebugDraw.h"
#include <stdio.h>

spCollisionMatrix 
_spCollisionMatrix()
{
    spCollisionMatrix matrix;

    matrix.CollideFunc[0][0] = spCollideCircles;
    matrix.CollideFunc[1][0] = spCollideCirclePolygon;
    matrix.CollideFunc[2][0] = spCollideCircleChain;
    
    matrix.CollideFunc[0][1] = spCollidePolygonCircle;
    matrix.CollideFunc[1][1] = spCollidePolygons;
    matrix.CollideFunc[2][1] = spCollidePolygonChain;

    matrix.CollideFunc[0][2] = spCollideChainCircle;
    matrix.CollideFunc[1][2] = spCollideChainPolygon;
    matrix.CollideFunc[2][2] = 0;

    return matrix;
}

spCollisionFunc 
spCollisionQueryFunc(
    const spCollisionMatrix& matrix, 
    const spShapeType        type_a, 
    const spShapeType        type_b)
{
    spCollisionFunc collision_func = matrix.CollideFunc[type_b][type_a];
    spAssert(collision_func != 0, "trying to collide two chains!");
    return collision_func;
}

spCollisionInput 
_spCollisionInput(
    const spShape* sa, 
    const spShape* sb, 
    const spTransform* xfa, 
    const spTransform* xfb)
{
    return
    {
        sa->type, /// type_a
        sb->type, /// type_b
        sa,       /// shape_a
        sb,       /// shape_b
        xfa,      /// transform_a
        xfb       /// transform_b
    };
}

spCollisionInput 
spCollisionInputSwap(const spCollisionInput& data)
{
    return spCollisionInput(data.shape_b, data.shape_a, data.transform_b, data.transform_a);
}

spBool 
spCollideCirclePolygon(spContact*& contact, const spCollisionInput& data)
{
    spBool result = spCollidePolygonCircle(contact, spCollisionInputSwap(data));
    return result;
}

spBool
spCollideEdgeCircle(const spEdge* edge, const spCircle* circle)
{
    return spFalse;
}

spBool 
spCollidePolygonCircle2(spContact*& contact, const spCollisionInput& data)
{
    const spPolygon* poly = (spPolygon*)data.shape_a;
    const spCircle* circle = (spCircle*)data.shape_b;
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spMaterial* ma = &poly->base_class.material;
    const spMaterial* mb = &circle->base_class.material;
    const spVector cp = spMult(*xfa, poly->base_class.bound.center);
    const spVector cc = spMult(*xfb, circle->base_class.bound.center);
    const spFloat cr2 = circle->radius * circle->radius;
    const spInt edge_count = poly->count;

    spFloat dist = SP_MIN_FLT;
    spEdge* edges = poly->edges;

    for (spInt i = 0; i < edge_count; ++i)
    {
        spEdge* edge = edges + i;
    }

    if (spDistanceSquared(cp, cc) < cr2)
    {
        /// TODO:
        /// find
    }
    return spFalse;
}

#define DRAW_POINT(point, r, g, b) \
        glPointSize(5.0f); \
        glBegin(GL_POINTS); \
        glColor3f(r, g, b); \
        glVertex2f(point.x, point.y); \
        glEnd(); \
        glColor3f(1.0f, 1.0f, 1.0f); \
        glPointSize(1.0f); \

spBool 
spCollidePolygonCircle(spContact*& contact, const spCollisionInput& data)
{
    /// box2d poly / circle collision alg. *lines 51 - 154*
    /// https://github.com/ansman/box2d/blob/master/Box2D/Box2D/Collision/b2CollideCircle.cpp 
    const spPolygon* poly = (spPolygon*)data.shape_a;
    const spCircle* circle = (spCircle*)data.shape_b;
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spMaterial* ma = &poly->base_class.material;
    const spMaterial* mb = &circle->base_class.material;

    spVector c  = spMult(*xfb, circle->center);
    spVector cl = spTMult(*xfa, c);
    spFloat radius = poly->base_class.bound.radius + circle->base_class.bound.radius;
    spFloat max_sep = SP_MIN_FLT;
    spInt count = poly->count;
    spInt index = 0;
    spEdge* edges = poly->edges;

    for (spInt i = 0; i < count; ++i)
    {
        spFloat sep = spDot(edges[i].normal, spSub(cl, edges[i].vertex));

        if (sep > radius) return spFalse;

        if (sep > max_sep)
        {
            max_sep = sep;
            index = i;
        }
    }

    spLog("%.7f\n", max_sep);

    spVector v0 = edges[index].vertex;
    spVector v1 = edges[(index+1) % count].vertex;

    if (max_sep < SP_EPSILON)
    {
        spLog("INSIDE<0\n");
        spVector pa = spMult(*xfa, poly->base_class.bound.center);
        spVector pb = spMult(*xfb, circle->base_class.bound.center);
        glPointSize(10.0f);
        spVector point = spMult(spAdd(pa, pb), 0.5f);
        DRAW_POINT(point, 1.0f, 0.0f, 1.0f);
        contact->points[0].r_a = spSub(point, pa);
        contact->points[0].r_b = spSub(point, pb);
        contact->normal = spMult(xfa->q, spSub(cl, v0));
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->count = 1;
        spNormalize(&contact->normal);;
    }

    spFloat p0 = spDot(spSub(cl, v0), spSub(v1, v0));
    spFloat p1 = spDot(spSub(cl, v1), spSub(v0, v1));

    if (p0 <= 0.0f)
    {
        if (spDistanceSquared(cl, v0) > radius * radius)
        {
            return spFalse;
        }

        spLog("P0<0\n");
        spVector pa = spMult(*xfa, poly->base_class.bound.center);
        spVector pb = spMult(*xfb, circle->base_class.bound.center);
        spVector point = spMult(*xfa, v0);
        point = v0;
        DRAW_POINT(point, 1.0f, 0.0f, 0.0f);
        contact->points[0].r_a = spSub(point, pa);
        contact->points[0].r_b = spSub(point, pb);
        contact->normal = spMult(xfa->q, spSub(cl, v0));
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->count = 1;
        spNormalize(&contact->normal);
    }
    else if (p1 <= 0.0f)
    {
        if (spDistanceSquared(cl, v1) > radius * radius)
        {
            return spFalse;
        }

        spLog("P1<0\n");
        spVector pa = spMult(*xfa, poly->base_class.bound.center);
        spVector pb = spMult(*xfb, circle->base_class.bound.center);
        spVector point = spMult(*xfa, v1);
        point = v1;
        DRAW_POINT(point, 0.0f, 1.0f, 0.0f);
        contact->points[0].r_a = spSub(point, pa);
        contact->points[0].r_b = spSub(point, pb);
        contact->normal = spMult(xfa->q, spSub(cl, v1));
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->count = 1;
        spNormalize(&contact->normal);;
    }
    else
    {
        spLog("ELSE<0\n");
        spVector fc = spMult(spAdd(v0, v1), 0.5f);
        spVector n = edges[index].normal;
        spFloat s = spDot(spSub(cl, fc), n);
        if (s > radius) return spFalse;

        spVector pa = spMult(*xfa, poly->base_class.bound.center);
        spVector pb = spMult(*xfb, circle->base_class.bound.center);
        spVector point = spMult(n, circle->radius);
        DRAW_POINT(point, 0.0f, 0.0f, 1.0f);
        contact->points[0].r_a = spSub(point, pa);
        contact->points[0].r_b = spSub(point, pb);
        contact->normal = spMult(xfa->q, n);
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->count = 1;
        spNormalize(&contact->normal);;
    }

    return spTrue;
}

spBool 
spCollideCircleChain(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

spBool 
spCollideChainCircle(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

spBool 
spCollidePolygonChain(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

spBool 
spCollideChainPolygon(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

spBool 
spCollideCircles(spContact*& contact, const spCollisionInput& data)
{
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spCircle* ca = (spCircle*) data.shape_a;
    const spCircle* cb = (spCircle*) data.shape_b;

    spVector world_a  = spMult(*xfa, ca->center);
    spVector world_b  = spMult(*xfb, cb->center);
    spVector distance = spSub(world_b, world_a);
    spFloat distance2 = spDot(distance, distance);
    spFloat radius    = ca->radius + cb->radius;
    spFloat radius2   = radius * radius;

    /// they arent in contact, return false;
    if (distance2 > radius2)
    {
        return spFalse;
    }

    /// they are in contact, create the contact and prepend it to the linked list
    spBody* ba = ca->base_class.body;
    spBody* bb = cb->base_class.body;
    const spMaterial* ma = &ca->base_class.material;
    const spMaterial* mb = &ca->base_class.material;

    /// create the contact
    spVector point = spMult(spAdd(world_a, world_b), 0.5f);

#ifdef SP_DEBUG_DRAW
    contact->points[0].p = point;
#endif
    contact->points[0].r_a = spSub(point, world_b);
    contact->points[0].r_b = spSub(point, world_a);
    contact->normal = spSub(world_a, world_b);
    contact->friction = spMaterialComputeFriction(ma, mb);
    contact->restitution = spMaterialComputeRestitution(ma, mb);
    contact->count = 1;
    spNormalize(&contact->normal);
    spNegate(&contact->normal);

    return spTrue;
}

spBool 
spCollidePolygons(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}