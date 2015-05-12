
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
#define DRAW_POINT(point, r, g, b) \
        glPointSize(5.0f); \
        glBegin(GL_POINTS); \
        glColor3f(r, g, b); \
        glVertex2f(point.x, point.y); \
        glEnd(); \
        glColor3f(1.0f, 1.0f, 1.0f); \
        glPointSize(1.0f); \

#define DRAW_TRI(p0, p1, p2, r, g, b) \
    glBegin(GL_TRIANGLES); \
    glColor3f(r, g, b); \
    glVertex2f(p0.x, p0.y); \
    glVertex2f(p1.x, p1.y); \
    glVertex2f(p2.x, p2.y); \
    glColor3f(1.0f, 1.0f, 1.0f); \
    glEnd(); \

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
spCollidePolygonCircle(spContact*& contact, const spCollisionInput& data)
{
    const spPolygon* poly = (spPolygon*)data.shape_a;
    const spCircle* circle = (spCircle*)data.shape_b;
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spMaterial* ma = &poly->base_class.material;
    const spMaterial* mb = &circle->base_class.material;
    spVector ca = spMult(*xfa, poly->base_class.bound.center);
    spVector cb = spMult(*xfb, circle->center);
    spVector lca = poly->base_class.bound.center;
    spVector lcb = circle->center;
    spFloat ra = poly->base_class.bound.radius;
    spFloat rb = circle->base_class.bound.radius;

    spVector c  = spMult(*xfb, lcb);
    spVector lc = spTMult(*xfa, c);

    spInt normalIndex = 0;
    spFloat separation = SP_MIN_FLT;
    spFloat radius = poly->base_class.bound.radius + circle->radius;
    radius = circle->radius;
    spInt vertexCount = poly->count;
    spEdge* edges = poly->edges;

    for (spInt i = 0; i < vertexCount; ++i)
    {
        spFloat s = spDot(edges[i].normal, spSub(lc, edges[i].vertex));

        if (s > radius)
        {
            return spFalse;
        }

        if (s > separation)
        {
            separation = s;
            normalIndex = i;
        }
    }

    spInt vi1 = normalIndex;
    spInt vi2 = (vi1 + 1) % vertexCount;

    spVector v1 = edges[vi1].vertex;
    spVector v2 = edges[vi2].vertex;

    if (separation < SP_FLT_EPSILON)
    {
        /// TODO: Fix
        spVector localNormal = edges[vi1].normal;
        spVector localPoint = spMult(spAdd(v1, v2), 0.5f);
        spVector localPoint_0 = circle->center;

        spVector planePoint = spMult(*xfa, localPoint);
        spVector clipPoint = spMult(*xfb, localPoint_0);
        spVector cA = spAdd(clipPoint, spMult(localNormal, (ra - spDot(spSub(clipPoint, planePoint), localNormal))));
        spVector cB = spSub(clipPoint, spMult(rb, localNormal));
        spVector point = spMult(spAdd(cA, cB), 0.5f);
        DRAW_POINT(cA, 1.0f, 0.0f, 1.0f);
        DRAW_POINT(cB, 1.0f, 0.0f, 1.0f);
        DRAW_POINT(point, 1.0f, 0.0f, 1.0f);

        contact->count = 1;
        contact->normal = spMult(xfa->q, localNormal);
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);
        spNormalize(&contact->normal);
        spNegate(&contact->normal);

        spLog("sep\n");
        return spTrue;
    }

    spFloat u1 = spDot(spSub(lc, v1), spSub(v2, v1));
    spFloat u2 = spDot(spSub(lc, v2), spSub(v1, v2));

    if (u1 <= 0.0f)
    {
        if (spDistanceSquared(lc, v1) > radius * radius)
        {
            return spFalse;
        }
        spVector localNormal = spSub(v1, lc);

        spVector point = spMult(*xfa, v1);
        spVector normal = spMult(xfa->q, spSub(v1, lc));
        spNormalize(&normal);
        DRAW_POINT(point, 1.0f, 0.0f, 0.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);

        spLog("u1\n");
    }
    else if (u2 <= 0.0f)
    {
        if (spDistanceSquared(lc, v2) > radius * radius)
        {
            return spFalse;
        }

        spVector point = spMult(*xfa, v2);
        spVector normal = spMult(xfa->q, spSub(v2, lc));
        spNormalize(&normal);
        DRAW_POINT(point, 0.0f, 1.0f, 0.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);

        spLog("u2\n");
    }
    else
    {
        spVector faceCenter = spMult(spAdd(v1, v2), 0.5f);
        spFloat sep = spDot(spSub(lc, faceCenter), edges[vi1].normal);
        if (sep > radius)
        {
            return spFalse;
        }

        spVector normal = spMult(xfa->q, edges[vi1].normal);
        spNormalize(&normal);
        spNegate(&normal);
        spVector point = spAdd(spMult(normal, rb), cb);
        DRAW_POINT(point, 0.0f, 0.0f, 1.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);

        spLog("else\n");
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
    contact->normal = spSub(world_b, world_a);
    contact->friction = spMaterialComputeFriction(ma, mb);
    contact->restitution = spMaterialComputeRestitution(ma, mb);
    contact->count = 1;
    spNormalize(&contact->normal);

    return spTrue;
}

spBool 
spCollidePolygons(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}