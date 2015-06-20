
#include "spCollision.h"
#include "spContact.h"
#include "spDebugDraw.h"
#include <stdio.h>
#include "spBody.h"

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

spMinkowskiPoint 
spMinkowskiPointConstruct(spVector* a, spVector* b)
{
    spMinkowskiPoint m;
    m.v = spSub(*a, *b);
    m.a = *a;
    m.b = *b;
    return m;
}

spBool 
spCollideCirclePolygon(spContact*& contact, const spCollisionInput& data)
{
    //spBool result = spCollidePolygonCircle(contact, spCollisionInputSwap(data));
    //spLog("SWAP");
    //spNegate(&contact->normal);
    ////spVector tmp = contact->points[0].r_a;
    ////contact->points[0].r_a = contact->points[0].r_b;
    ////contact->points[0].r_a = tmp;
    //return result;
    glPointSize(12.0f);
    const spPolygon* poly = (spPolygon*)data.shape_b;
    const spCircle* circle = (spCircle*)data.shape_a;
    const spTransform* xfa = data.transform_b;
    const spTransform* xfb = data.transform_a;
    const spMaterial* ma = &poly->base_class.material;
    const spMaterial* mb = &circle->base_class.material;
    spVector ca = spMult(*xfa, poly->base_class.bound.center);
    spVector cb = spMult(*xfb, circle->center);
    spVector lca = poly->base_class.bound.center;
    spVector lcb = circle->center;
    spFloat rb = poly->base_class.bound.radius;
    spFloat ra = circle->base_class.bound.radius;

    spVector c  = spMult(*xfb, lcb);
    spVector lc = spTMult(*xfa, c);

    spInt normalIndex = 0;
    spFloat separation = SP_MIN_FLT;
    spFloat radius = circle->radius;
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

    }

    spFloat u1 = spDot(spSub(lc, v1), spSub(v2, v1));
    spFloat u2 = spDot(spSub(lc, v2), spSub(v1, v2));

    if (u1 <= 0.0f)
    {
        if (spDistanceSquared(v1, lc) > radius * radius)
        {
            return spFalse;
        }

        spVector point = spMult(*xfa, v1);
        spVector normal = spMult(xfa->q, spSub(lc, v1)); ///changed v1,lc
        spNormalize(&normal);
        DRAW_POINT(point, 1.0f, 0.0f, 0.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_b = spSub(point, ca);
        contact->points[0].r_a = spSub(point, cb);
        contact->points[0].p = point;
        spLog("v1\n");
    }
    else if (u2 <= 0.0f)
    {
        if (spDistanceSquared(lc, v2) > radius * radius)
        {
            return spFalse;
        }

        spVector point = spMult(*xfa, v2);
        spVector normal = spMult(xfa->q, spSub(lc, v2)); 
        spNormalize(&normal);
        DRAW_POINT(point, 0.0f, 1.0f, 0.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_b = spSub(point, ca);
        contact->points[0].r_a = spSub(point, cb);
        contact->points[0].p = point;
        spLog("v2\n");
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
        spVector point = spAdd(spMult(spNegate(normal), ra), data.shape_a->body->p);
        DRAW_POINT(point, 0.0f, 0.0f, 1.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_b = spSub(point, ca);
        contact->points[0].r_a = spSub(point, cb);
        contact->points[0].p = point;
        spLog("n\n");
    }

    spNegate(&contact->normal);

    return spTrue;
}

spBool
spCollideEdgeCircle(const spEdge* edge, const spCircle* circle)
{
    return spFalse;
}

spBool 
spCollidePolygonCircle(spContact*& contact, const spCollisionInput& data)
{
    glPointSize(12.0f);
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
    spFloat radius = circle->radius;
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
        //spVector localNormal = edges[vi1].normal;
        //spVector localPoint = spMult(spAdd(v1, v2), 0.5f);
        //spVector localPoint_0 = circle->center;

        //spVector planePoint = spMult(*xfa, localPoint);
        //spVector clipPoint = spMult(*xfb, localPoint_0);
        //spVector cA = spAdd(clipPoint, spMult(localNormal, (ra - spDot(spSub(clipPoint, planePoint), localNormal))));
        //spVector cB = spSub(clipPoint, spMult(rb, localNormal));
        //spVector point = spMult(spAdd(cA, cB), 0.5f);
        //DRAW_POINT(cA, 1.0f, 0.0f, 1.0f);
        //DRAW_POINT(cB, 1.0f, 0.0f, 1.0f);
        //DRAW_POINT(point, 1.0f, 0.0f, 1.0f);

        //contact->count = 1;
        //contact->normal = spMult(xfa->q, localNormal);
        //contact->friction = spMaterialComputeFriction(ma, mb);
        //contact->restitution = spMaterialComputeRestitution(ma, mb);
        //contact->points[0].r_a = spSub(point, ca);
        //contact->points[0].r_b = spSub(point, cb);
        //spNormalize(&contact->normal);
        //spNegate(&contact->normal);

        //spLog("sep\n");
        //return spTrue;
    }

    spFloat u1 = spDot(spSub(lc, v1), spSub(v2, v1));
    spFloat u2 = spDot(spSub(lc, v2), spSub(v1, v2));

    if (u1 <= 0.0f)
    {
        if (spDistanceSquared(lc, v1) > radius * radius)
        {
            return spFalse;
        }

        spVector point = spMult(*xfa, v1);
        spVector normal = spMult(xfa->q, spSub(lc, v1)); ///changed v1,lc
        spNormalize(&normal);
        DRAW_POINT(point, 1.0f, 0.0f, 0.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);
        contact->points[0].p = point;
        spLog("v1\n");
    }
    else if (u2 <= 0.0f)
    {
        if (spDistanceSquared(lc, v2) > radius * radius)
        {
            return spFalse;
        }

        spVector point = spMult(*xfa, v2);
        spVector normal = spMult(xfa->q, spSub(lc, v2)); 
        spNormalize(&normal);
        DRAW_POINT(point, 0.0f, 1.0f, 0.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);
        contact->points[0].p = point;
        spLog("v2\n");
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
        spVector point = spAdd(spMult(spNegate(normal), rb), data.shape_b->body->p);
        DRAW_POINT(point, 0.0f, 0.0f, 1.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);
        contact->points[0].p = point;
        spLog("n\n");
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

    const spMaterial* ma = &ca->base_class.material;
    const spMaterial* mb = &ca->base_class.material;
    spBody* ba = ca->base_class.body;
    spBody* bb = cb->base_class.body;
    spVector point = spLerp(world_a, world_b, ca->radius / radius);

#ifdef SP_DEBUG_DRAW
    contact->points[0].p = point;
#endif
    contact->points[0].r_a = spSub(point, world_a);
    contact->points[0].r_b = spSub(point, world_b);
    contact->normal = spSub(world_b, world_a);
    contact->friction = spMaterialComputeFriction(ma, mb);
    contact->restitution = spMaterialComputeRestitution(ma, mb);
    contact->count = 1;
    spNormalize(&contact->normal);

    return spTrue;
}

spClosestPoints 
_spClosestPoints(const spVector* a, const spVector* b)
{
    spClosestPoints cp;
    cp.a = *a;
    cp.b = *b;
    return cp;
}

spInt
spExtremalQueryIndex(const spPolygon* poly, const spTransform* xf, const spVector& normal)
{
    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count = poly->count;

    spFloat mproj = -SP_MAX_FLT; /// largest projection
    spInt mi = 0; /// max index with the largest projection

    /// find the most extreme point along a direction
    for (spInt i = 0; i < count; ++i)
    {
        spEdge* edge = edges + i;
        spVector v = spMult(*xf, edge->vertex);
        spFloat proj = spDot(v, normal);

        /// this projection is larger, save its info
        if (proj > mproj)
        {
            mproj = proj;
            mi = i;
        }
    }

    /// return the extremal index of the poly along a direction
    return mi;
}

spVector
spExtremalQuery(const spPolygon* poly, const spTransform* xf, const spVector& normal)
{
    spEdge* edges = poly->edges;
    spInt index = spExtremalQueryIndex(poly, xf, normal);
    return spMult(*xf, edges[index].vertex);
}

static spMinkowskiPoint
spSupportPoint(const spPolygon* a, const spPolygon* b, const spTransform* xfa, const spTransform* xfb, const spVector& normal)
{
    spVector va = spExtremalQuery(a, xfa, normal);
    spVector vb = spExtremalQuery(b, xfb, spNegate(normal));
    return spMinkowskiPointConstruct(&va, &vb);
}

static inline spBool
spOriginToLeft(spVector a, spVector b)
{
    return (a.x - b.x) * (a.y + b.y) < (a.y - b.y) * (a.x + b.x);
}

static inline spBool
spOriginToRight(spVector a, spVector b)
{
    return (a.x - b.x) * (a.y + b.y) > (a.y - b.y) * (a.x + b.x);
}

static inline spFloat
spClosestT(spVector t, spVector h)
{
    spVector M = spSub(h, t);
    return spClamp(spDot(M, spNegate(t))/spLengthSquared(M), 0.0f, 1.0f);
}

static inline spVector
spClosestPointToOriginOnEdge(spVector t, spVector h)
{
    /// http://www.geometrictools.com/Documentation/DistancePointLine.pdf
    /// P = origin (0, 0).
    /// B = a
    spVector M = spSub(h, t);
    spFloat t0 = spClamp(spDot(M, spNegate(t))/spLengthSquared(M), 0.0f, 1.0f);
    return spNegate(spAdd(t, spMult(M, t0)));
}

static inline spFloat
spDistanceToOriginFromEdge(spVector t, spVector h)
{
    /// t = vector tail
    /// h = vector head
    /// vector = sub(head, tail)
    return spLengthSquared(spClosestPointToOriginOnEdge(t, h));
}

///#define SP_DEBUG_DRAW_EPA

void spDebugDrawHull(spMinkowskiPoint* hull, spInt count)
{
    for (spInt t = count-1, h = 0; h < count; t = h, h++)
    {
        spDebugDrawLine(hull[h].v, hull[t].v, spYellow(0.1f));
    }
}

struct spContactEdge
{
    spVector t;
    spVector h;
    spVector n;
};

static spContactEdge
spEPAContactEdge(spPolygon* poly, const spTransform* xf, spVector n)
{
    spContactEdge ce;
    spEdge* e = poly->edges;
    spInt c = poly->count;
    spInt i1  = spExtremalQueryIndex(poly, xf, n);
    //spInt i0 = i1 == 0   ? c-1 : i1-1;
    //spInt i2 = i1 == c-1 ?   0 : i1+1;
    spInt i0 = (i1 - 1 + c) % c;
    spInt i2 = (i1 + 1) % c;

    /// initialize the contact edge
    if (spDot(n, spMult(xf->q, e[i1].normal)) > spDot(n, spMult(xf->q, e[i2].normal)))
    {
        ce.t = spMult(*xf, e[i0].vertex);
        ce.h = spMult(*xf, e[i1].vertex);
        ce.n = spMult(xf->q, e[i1].normal);
    }
    else
    {
        ce.t = spMult(*xf, e[i1].vertex);
        ce.h = spMult(*xf, e[i2].vertex);
        ce.n = spMult(xf->q, e[i2].normal);
    }
    return ce;
}

static void
spMaxPenetrationContact(spContact*& contact, spFloat a, spFloat b)
{
}

static void
spEPAContactPoints(spContact*& contact, spMinkowskiPoint* head, spMinkowskiPoint* tail, const spCollisionInput& data)
{
    // get the polygons polygons
    spPolygon* pa = (spPolygon*)data.shape_a;
    spPolygon* pb = (spPolygon*)data.shape_b;
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spMaterial* ma = &pa->base_class.material;
    const spMaterial* mb = &pa->base_class.material;

    /// compute the lerp t bettwen the two minkowski points
    spFloat t = spClosestT(tail->v, head->v);
    spFloat pen = spDistanceToOriginFromEdge(tail->v, head->v);

    /// get the points in world space
    spVector wa = spLerp(tail->a, head->a, t); 
    spVector wb = spLerp(tail->b, head->b, t);
    spVector ba = spMult(*xfa, pa->base_class.bound.center);
    spVector bb = spMult(*xfb, pb->base_class.bound.center);

    /// get the vector of the two world points
    spVector v = spSub(head->v, tail->v);

    /// calculate the contact normal
    spVector n = spSkewT(v);
    spNormalize(&n);

    spInt count = contact->count;
    if (count == 2)
    {
        struct spRelativeVelocity
        {
            spVector r_a;
            spVector r_b;
            spFloat pen;
        };

        spRelativeVelocity rv[4] = {
           { spSub(wa, ba), spSub(wa, bb), spDot(wa, n) },
           { spSub(wb, ba), spSub(wb, bb), spDot(wb, n) },
           { contact->points[0].r_a, contact->points[0].r_b, spDot(contact->points[0].p, n) },
           { contact->points[1].r_a, contact->points[1].r_b, spDot(contact->points[1].p, n) } 
        };

        for (spInt i = 0; i < 4; ++i)
        {
            for (spInt j = i; j < 4; j++)
            {
                if (rv[i].pen > rv[j].pen)
                {
                    spRelativeVelocity tmp = rv[i];
                    rv[i] = rv[j];
                    rv[j] = tmp;
                }
            }
        }

        contact->points[0].r_a = rv[0].r_a;
        contact->points[0].r_b = rv[0].r_b;
        contact->points[1].r_a = rv[1].r_a;
        contact->points[1].r_b = rv[1].r_b;
        contact->normal = n;
        contact->pen = rv[0].pen;
    }
    else
    {
        contact->count = 2;
        contact->normal = n;
        contact->points[0].p = wa;
        contact->points[1].p = wb;
        contact->points[0].r_a = spSub(wa, ba);
        contact->points[0].r_b = spSub(wa, bb);
        contact->points[1].r_a = spSub(wb, ba);
        contact->points[1].r_b = spSub(wb, bb);
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->pen = spMax(spDot(wa, n), spDot(wb, n));
    }

    contact->pen = pen;
    return;
}

static void
spEPA(spContact*& contact, spMinkowskiPoint* m0, spMinkowskiPoint* m1, spMinkowskiPoint* m2, const spCollisionInput& data)
{
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spPolygon*   pa  = (spPolygon*)data.shape_a;
    const spPolygon*   pb  = (spPolygon*)data.shape_b;

    spMinkowskiPoint  hull0[24] = { 0 };
    spMinkowskiPoint  hull1[24] = { 0 };
    spMinkowskiPoint* hull = NULL;
    spInt count = 3;

    hull0[0] = *m0;
    hull0[1] = *m1;
    hull0[2] = *m2;
    hull = hull0;

    static const spInt max_iters = 21;

    for (spInt iter = 0; iter < max_iters; ++iter)
    {
        spFloat min_dist = SP_MAX_FLT; /// min distance to an edge
        spInt ti = 0; /// tail index
        spInt hi = 0; /// head index
        for (spInt t = count-1, h = 0; h < count; t = h, h++)
        {
            /// find the closest edge to the origin
            spFloat d = spDistanceToOriginFromEdge(hull[t].v, hull[h].v);
            if (d < min_dist)
            {
                min_dist = d;
                ti = t;
                hi = h;
            }
        }

        spMinkowskiPoint head = hull[hi];
        spMinkowskiPoint tail = hull[ti];

        /// get the new point on the hull
        spVector dir = spSkewT(spSub(head.v, tail.v));
        spMinkowskiPoint m = spSupportPoint(pa, pb, xfa, xfb, dir);

        /// check if the point it already in the hull
        for (spInt i = 0; i < count; ++i)
        {
            if (spEqual(hull[i].v, m.v))
            {
                //spDebugDrawHull(hull, count);
                spEPAContactPoints(contact, &head, &tail, data);
                return;
            }
        }

        /// rebuild the hull
        spMinkowskiPoint* hull_old;
        spMinkowskiPoint* hull_new;

        /// check if we are on an odd number iter
        if (iter & 1) 
        {
            hull_old = hull1; 
            hull_new = hull0;
        }
        else
        {
            hull_old = hull0;
            hull_new = hull1;
        }

        count++;
        /// copy the hull over
        for (spInt n = 0, o = 0; n < count; ++n)
        {
            if (n == hi)
            {
                hull_new[n] = m;
            }
            else
            {
                hull_new[n] = hull_old[o++];
            }
        }
        hull = hull_new;
        if (iter == 28) spLog("%d\n", iter);
    }
    //spDebugDrawHull(hull, count);
}

///#define SP_DEBUG_DRAW_GJK

static spBool 
spGJK(spContact*& contact, const spCollisionInput& data)
{
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spPolygon*   pa  = (spPolygon*)data.shape_a;
    const spPolygon*   pb  = (spPolygon*)data.shape_b;
    const spBound*     ba  = &pa->base_class.bound;
    const spBound*     bb  = &pb->base_class.bound;

    /// generate an initial axis direction for support points
    spVector bca = spMult(*xfa, spBoundGetCenter(ba));
    spVector bcb = spMult(*xfb, spBoundGetCenter(bb));
    spVector dir = spSkew(spSub(bca, bcb));

    /// calculate initial minkowski points
    spMinkowskiPoint m0 = spSupportPoint(pa, pb, xfa, xfb, dir);
    spMinkowskiPoint m1 = spSupportPoint(pa, pb, xfa, xfb, spNegate(dir));

    static const spInt max_iters = 16;
    for (spInt i = 0; i < max_iters; ++i)
    {
        /// make sure the origin is always to the left of the edge
        if (spOriginToRight(m0.v, m1.v)) 
        {
            spSwap(&m0.v, &m1.v);
        }
            
        /// calculate a new direction for the next support point
        spVector dir = spSkew(spSub(m0.v, m1.v));

        /// expand the simplex by generating a new support point
        spMinkowskiPoint m2 = spSupportPoint(pa, pb, xfa, xfb, dir);

#ifdef SP_DEBUG_DRAW_GJK
        /// draw the origin
        spDebugDrawPoint(spVectorZero(), spGreen(1.0f));

        /// draw the edge normal of the first 2 simplex points
        spVector p = spLerp(m0.v, m1.v, 0.5f);
        spNormalize(&dir);
        spDebugDrawLine(p, spAdd(p, dir), spGreen(0.25f));

        ///// draw the simplex and its edges
        spDebugDrawTriangle(m0.v, m1.v, m2.v, spYellow(0.05f));
        spDebugDrawLine(m0.v, m1.v, spYellow(0.15f));
        spDebugDrawLine(m1.v, m2.v, spYellow(0.15f));
        spDebugDrawLine(m2.v, m0.v, spYellow(0.15f));
#endif

        /// check if the origin is inside of the 3-simplex or the new minkowski point is on origin
        if (spOriginToLeft(m1.v, m2.v) && spOriginToLeft(m2.v, m0.v) || spEqual(m2.v, spVectorZero()))
        {
            /// the origin is in the simplex, pass the 3-simplex to EPA and generate contact info
            spEPA(contact, &m0, &m2, &m1, data);
            return spTrue;
        }
        else
        {
            /// check if this is the closest edge to the origin.
            if (spMax(spDot(m0.v, dir), spDot(m1.v, dir)) >= spDot(m2.v, dir))
            {
#ifdef SP_DEBUG_DRAW_GJK
                spFloat t = spClosestT(m0.v, m1.v);
                spVector pa = spLerp(m0.a, m1.a, t);
                spVector pb = spLerp(m0.b, m1.b, t);
                spDebugDrawLine(m0.v, m1.v, spYellow(0.5f));
                spDebugDrawLine(pa, pb, spGreen(0.8f));
                spDebugDrawPoint(pa, spGreen(1.0f));
                spDebugDrawPoint(pb, spGreen(1.0f));
#endif
                return spFalse;
            }
            else
            {
                /// check which edge contains a closer point to the origin
                /// remove the uneeded point from the simplex that is farther away
                if (spDistanceToOriginFromEdge(m0.v, m2.v) > spDistanceToOriginFromEdge(m1.v, m2.v))
                {
                    m0 = m2;
                }
                else
                {
                    m1 = m2;
                }
            }
        }
    }

    /// GJK terminated without converging and without an intersection.
    return spFalse;
}

spBool 
spCollidePolygons(spContact*& contact, const spCollisionInput& data)
{
    return spGJK(contact, data);
}