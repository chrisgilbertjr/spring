
#include "spCollision.h"
#include "spContact.h"
#include "spDebugDraw.h"
#include <stdio.h>
#include "spBody.h"

static inline void
initContact(spContact* contact, spFloat count, spVector normal, spVector point, spVector pointA, spVector pointB, const spCollisionInput& data)
{
    const spMaterial* mA = &data.shape_a->material;
    const spMaterial* mB = &data.shape_b->material;

    contact->count = count;
    contact->normal = normal;
    contact->points[0].r_a = spSub(point, pointA);
    contact->points[0].r_b = spSub(point, pointB);
    contact->points[0].p = point;
    contact->friction = spMaterialComputeFriction(mA, mB);
    contact->restitution = spMaterialComputeRestitution(mA, mB);
}

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
spCollisionQueryFunc(const spCollisionMatrix& matrix, const spShapeType type_a, const spShapeType type_b)
{
    spCollisionFunc collision_func = matrix.CollideFunc[type_b][type_a];
    spAssert(collision_func != 0, "trying to collide two chains!");
    return collision_func;
}

spCollisionInput 
_spCollisionInput(const spShape* sa, const spShape* sb, const spTransform* xfa, const spTransform* xfb)
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

spMinkowskiPoint 
spMinkowskiPointConstruct(spVector a, spVector b)
{
    spMinkowskiPoint m;
    m.v = spSub(a, b);
    m.a = a;
    m.b = b;
    return m;
}

spBool 
spCollideCirclePolygon(spContact*& contact, const spCollisionInput& data)
{
    /// swap the collision data, and do the collision
    spBool result = spCollidePolygonCircle(contact, spCollisionInputSwap(data));

    /// swap the normal and relative velocities
    spNegate(&contact->normal);
    spVector tmp = contact->points[0].r_a;
    contact->points[0].r_a = contact->points[0].r_b;
    contact->points[0].r_b = tmp;

    /// return the result
    return result;
}

spBool
spCollideEdgeCircle(const spEdge* edge, const spCircle* circle)
{
    /// TODO:
    return spFalse;
}

spBool 
spCollidePolygonCircle(spContact*& contact, const spCollisionInput& data)
{
    /// get the shapes
    /// TODO: DO A SHAPE CAST HERE!
    const spPolygon* poly  = (spPolygon*)data.shape_a;
    const spCircle* circle = (spCircle*)data.shape_b;

    /// get the shapes transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

    /// get local centers
    /// TODO: ADD POLY CENTER, NOT COM, MULTI-BODIES WILL NOT WORK!
    spVector lcA = poly->base_class.body->com;
    spVector lcB = circle->center;

    /// compute centers in world space
    spVector cA = spMult(*xfA, lcA);
    spVector cB = spMult(*xfB, lcB);

    /// get circle radius, and radius^2
    spFloat radius  = circle->radius;
    spFloat radius2 = radius * radius;

    /// get local center b in a's local space
    spVector center = spTMult(*xfA, cB);

    spInt iSep = 0;
    spInt count = poly->count;
    spEdge* edges = poly->edges;
    spFloat maxSep = -SP_MAX_FLT;

    /// find the closest poly edge from the circle
    for (spInt i = 0; i < count; ++i)
    {
        spFloat edgeSep = spDot(edges[i].normal, spSub(center, edges[i].vertex));

        if (edgeSep > radius)
        {
            return spFalse;
        }

        if (edgeSep > maxSep)
        {
            maxSep = edgeSep;
            iSep = i;
        }
    }

    /// get the indices of the closest edge vertices
    spInt i0 = iSep;
    spInt i1 = (i0 + 1) % count;

    /// get the edge vertices
    spVector v0 = edges[i0].vertex;
    spVector v1 = edges[i1].vertex;

    /// the circles center is inside of the polygon
    /// TODO:
    if (maxSep < SP_FLT_EPSILON)
    {
        /// TODO:
    }

    /// compute voronoi regions
    spFloat voronoi0 = spDot(spSub(center, v0), spSub(v1, v0));
    spFloat voronoi1 = spDot(spSub(center, v1), spSub(v0, v1));

    /// collision normal and point
    spVector normal, point;

    /// the point is to the left and in v0's voronoi region
    if (voronoi0 <= 0.0f)
    {
        /// check if the point is inside of the circle
        if (spDistanceSquared(center, v0) > radius2) return spFalse;

        /// compute the point and normal
        normal = spNormal(spMult(xfA->q, spSub(center, v0)));
        point  = spMult(*xfA, v0);
    }

    /// the point is to the right and in v1's voronoi region
    else if (voronoi1 <= 0.0f)
    {
        /// check if the point is inside of the circle
        if (spDistanceSquared(center, v1) > radius2) return spFalse;

        normal = spNormal(spMult(xfA->q, spSub(center, v1)));
        point  = spMult(*xfA, v1);
    }
    
    /// the point is in between v1 and v2, its on the edges voronoi region
    else
    {
        /// check if the point is inside of the circle
        if (spDot(spSub(center, v0), edges[i0].normal) > radius) return spFalse;

        normal = spNormal(spMult(xfA->q, edges[i0].normal));
        point  = spAdd(spMult(spNegate(normal), radius), data.shape_b->body->p);
    }

    /// initialize the contact and return a successful collision
    initContact(contact, 1, normal, point, cA, cB, data);
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
    return spMinkowskiPointConstruct(va, vb);
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

static inline spFloat
spDistanceToOriginFromEdge2(spVector t, spVector h)
{
    /// t = vector tail
    /// h = vector head
    /// vector = sub(head, tail)
    return spLength(spClosestPointToOriginOnEdge(t, h));
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
    spFloat pen = spDistanceToOriginFromEdge2(tail->v, head->v);

    /// get the points in world space
    spVector wa = spLerp(tail->a, head->a, t); 
    spVector wb = spLerp(tail->b, head->b, t);
    spVector ba = spMult(*xfa, pa->base_class.body->com);
    spVector bb = spMult(*xfb, pb->base_class.body->com);

    /// get the vector of the two world points
    spVector v = spSub(head->v, tail->v);

    /// calculate the contact normal
    spVector n = spNormal(spSkewT(v));

    spInt count = contact->count;
    spMinkowskiPoint a = *head;
    spMinkowskiPoint b = *tail;

    static spInt index = 1;

    if (count == 2)
    {
        if (index == 0)
        {
            if (!spAlmostEqual(contact->points[1].p, wa, 0.1f))
        	{
        	    contact->count = index;
        		contact->normal = n;
        		contact->points[1].p = wa;
        		contact->points[1].r_a = spSub(wa, ba);
        		contact->points[1].r_b = spSub(wa, bb);
        		contact->friction = spMaterialComputeFriction(ma, mb);
        		contact->restitution = spMaterialComputeRestitution(ma, mb);
                contact->pen = pen;
        	}
            index = 1;
        }
        else if (index == 1)
        {
            if (!spAlmostEqual(contact->points[0].p, wa, 0.1f))
        	{
        	    contact->count = index;
        		contact->normal = n;
        		contact->points[0].p = wa;
        		contact->points[0].r_a = spSub(wa, ba);
        		contact->points[0].r_b = spSub(wa, bb);
        		contact->friction = spMaterialComputeFriction(ma, mb);
        		contact->restitution = spMaterialComputeRestitution(ma, mb);
                contact->pen = pen;
        	}
            index = 0;
        }
    }
    else if (count == 1)
    {
        if (!spAlmostEqual(contact->points[0].p, wa, 0.1f))
        {
            contact->count = 2;
        	contact->normal = n;
        	contact->points[1].p = wa;
        	contact->points[1].r_a = spSub(wa, ba);
        	contact->points[1].r_b = spSub(wa, bb);
        	contact->friction = spMaterialComputeFriction(ma, mb);
        	contact->restitution = spMaterialComputeRestitution(ma, mb);
            contact->pen = pen;
        }
    }
    else
    {
        contact->count = 1;
        contact->normal = n;
        contact->points[0].p = wa;
        contact->points[0].r_a = spSub(wa, ba);
        contact->points[0].r_b = spSub(wa, bb);
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->pen = pen;
    }
}

static void
spEPAContactPoints2(spContact*& contact, spMinkowskiPoint* head, spMinkowskiPoint* tail, const spCollisionInput& data)
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

    //spInt count = contact->count;
    //if (count == 2)
    //{
    //    struct spRelativeVelocity
    //    {
    //        spVector r_a;
    //        spVector r_b;
    //        spFloat pen;
    //    };

    //    spRelativeVelocity rv[4] = {
    //       { spSub(wa, ba), spSub(wa, bb), spDot(wa, n) },
    //       { spSub(wb, ba), spSub(wb, bb), spDot(wb, n) },
    //       { contact->points[0].r_a, contact->points[0].r_b, spDot(contact->points[0].p, n) },
    //       { contact->points[1].r_a, contact->points[1].r_b, spDot(contact->points[1].p, n) } 
    //    };

    //    for (spInt i = 0; i < 4; ++i)
    //    {
    //        for (spInt j = i; j < 4; j++)
    //        {
    //            if (rv[i].pen > rv[j].pen)
    //            {
    //                spRelativeVelocity tmp = rv[i];
    //                rv[i] = rv[j];
    //                rv[j] = tmp;
    //            }
    //        }
    //    }

    //    contact->points[0].r_a = rv[0].r_a;
    //    contact->points[0].r_b = rv[0].r_b;
    //    contact->points[1].r_a = rv[1].r_a;
    //    contact->points[1].r_b = rv[1].r_b;
    //    contact->normal = n;
    //    contact->pen = rv[0].pen;
    //}
    //else
    //{
    //    contact->count = 2;
    //    contact->normal = n;
    //    contact->points[0].p = wa;
    //    contact->points[1].p = wb;
    //    contact->points[0].r_a = spSub(wa, ba);
    //    contact->points[0].r_b = spSub(wa, bb);
    //    contact->points[1].r_a = spSub(wb, ba);
    //    contact->points[1].r_b = spSub(wb, bb);
    //    contact->friction = spMaterialComputeFriction(ma, mb);
    //    contact->restitution = spMaterialComputeRestitution(ma, mb);
    //    contact->pen = spMax(spDot(wa, n), spDot(wb, n));
    //}

    //contact->pen = pen;
    //return;
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