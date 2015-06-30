

#include "spCollision.h"
#include "spContact.h"
#include "spDebugDraw.h"
#include <stdio.h>
#include "spBody.h"

struct spSupportPoint
{
    spVector point;
};

struct Edge
{
    spVector a;
    spVector b;
};

struct MinkowskiEdge
{
    spMinkowskiPoint head;
    spMinkowskiPoint tail;
    spVector normal;
    spFloat distance;
};

struct SupportPointContext
{
    spShape* shapeA;
    spShape* shapeB;
    SupportPointFunc supportPointA;
    SupportPointFunc supportPointB;
};

static inline spFloat
spLerpRatio(spVector t, spVector h)
{
    /// http://www.geometrictools.com/Documentation/DistancePointLine.pdf
    spVector M = spSub(h, t);
    return spClamp(spDot(M, spNegate(t))/spLengthSquared(M), 0.0f, 1.0f);
}

static inline struct MinkowskiEdge
MinkowskiEdgeConstruct(spMinkowskiPoint* head, spMinkowskiPoint* tail)
{
    /// get the closest minkowski point
    spFloat t = spLerpRatio(tail->v, head->v);
    spVector point = spLerp(tail->v, head->v, t);

    /// calculate the contact normal and penetration distance
    spVector delta   = spSub(head->v, tail->v);
    spVector normal  = spNormal(spSkewT(delta));
    spFloat distance = spDot(normal, point);

    MinkowskiEdge edge;
    edge.head = *head;
    edge.tail = *tail;
    edge.normal = normal;
    edge.distance = distance;
    return edge;
}

static inline struct spCollisionResult
spCollisionResultConstruct()
{
    spCollisionResult result;
    result.colliding = spFalse;
    result.pointA[0] = spVectorZero();
    result.pointB[0] = spVectorZero();
    result.pointA[1] = spVectorZero();
    result.pointB[1] = spVectorZero();
    result.normal    = spVectorZero();
    result.count     = 0.0f;

    return result;
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

static spCollisionInput 
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

static spCollisionResult 
spCollidePolygonCircle(const spPolygon* poly, const spCircle* circle)
{
    spCollisionResult result;
    result.colliding = spFalse;
    return result;
    /// get the shapes
    //const spShape* polyShape   = data.shape_a;
    //const spShape* circleShape = data.shape_b;

    //const spPolygon* poly   = spShapeCastPolygon(polyShape);
    //const spCircle*  circle = spShapeCastCircle(circleShape);

    ///// get the shapes transforms
    //const spTransform* xfA = data.transform_a;
    //const spTransform* xfB = data.transform_b;

    ///// get local centers
    //spVector lcA = spShapeGetCenter(polyShape);
    //spVector lcB = spShapeGetCenter(circleShape);

    ///// compute centers in world space
    //spVector cA = spMult(*xfA, lcA);
    //spVector cB = spMult(*xfB, lcB);

    ///// get circle radius, and radius^2
    //spFloat radius  = circle->radius;
    //spFloat radius2 = radius * radius;

    ///// get local center b in a's local space
    //spVector center = spTMult(*xfA, cB);

    //spInt iSep = 0;
    //spInt count = poly->count;
    //spEdge* edges = poly->edges;
    //spFloat maxSep = -SP_MAX_FLT;

    ///// find the closest poly edge from the circle
    //for (spInt i = 0; i < count; ++i)
    //{
    //    spFloat edgeSep = spDot(edges[i].normal, spSub(center, edges[i].vertex));

    //    /// the separation is > radius, they are not colliding
    //    if (edgeSep > radius)
    //    {
    //        return spFalse;
    //    }

    //    if (edgeSep > maxSep)
    //    {
    //        maxSep = edgeSep;
    //        iSep = i;
    //    }
    //}

    ///// get the indices of the closest edge vertices
    //spInt i0 = iSep;
    //spInt i1 = (i0 + 1) % count;

    ///// get the edge vertices
    //spVector v0 = edges[i0].vertex;
    //spVector v1 = edges[i1].vertex;

    ///// collision normal and point
    //spVector normal, point;
    //spFloat penetration;

    ///// the circles center is inside of the polygon
    ///// TODO:
    //if (maxSep < SP_FLT_EPSILON)
    //{
    //    normal = spMult(xfA->q, edges[i0].normal);
    //    point = spAdd(cA, spMult(spNegate(normal), radius));
    //    penetration = radius;

    //    initContact(contact, 0, 1, point, cA, cB, normal, penetration, data);
    //    return spTrue;
    //}

    ///// compute contact penetration
    ////penetration = radius - maxSep;

    ///// compute voronoi regions
    //spFloat voronoi0 = spDot(spSub(center, v0), spSub(v1, v0));
    //spFloat voronoi1 = spDot(spSub(center, v1), spSub(v0, v1));
    //spFloat pen2 = radius - maxSep;

    ///// the point is to the left and in v0's voronoi region
    //if (voronoi0 <= 0.0f)
    //{
    //    spFloat distance2 = spDistanceSquared(center, v0);

    //    /// check if the point is inside of the circle
    //    if (distance2 > radius2) return spFalse;

    //    /// compute the point and normal
    //    penetration = radius - spsqrt(distance2);
    //    normal = spNormal(spMult(xfA->q, spSub(center, v0)));
    //    point  = spMult(*xfA, v0);
    //}

    ///// the point is to the right and in v1's voronoi region
    //else if (voronoi1 <= 0.0f)
    //{
    //    spFloat distance2 = spDistanceSquared(center, v1);

    //    /// check if the point is inside of the circle
    //    if (distance2 > radius2) return spFalse;

    //    penetration = radius - spsqrt(distance2);
    //    normal = spNormal(spMult(xfA->q, spSub(center, v1)));
    //    point  = spMult(*xfA, v1);
    //}
    
    ///// the point is in between v1 and v2, its on the edges voronoi region
    //else
    //{
    //    /// check if the point is inside of the circle
    //    if (spDot(spSub(center, v0), edges[i0].normal) > radius) return spFalse;

    //    penetration = radius - maxSep;
    //    normal = spNormal(spMult(xfA->q, edges[i0].normal));
    //    point  = spAdd(spMult(spNegate(normal), radius), data.shape_b->body->p);
    //}


    ///// initialize the contact and return a successful collision
    //initContact(contact, 0, 1, point, cA, cB, normal, penetration, data);
    //return spTrue;
}

static spCollisionResult 
spCollideCirclePolygon(const spCircle* circle, const spPolygon* poly)
{
    /// swap the collision data, and do the collision
    spCollisionResult result = spCollidePolygonCircle(poly, circle);

    /// swap the normal and relative velocities
    /// invertResult;

    /// return the result
    return result;
}

static spCollisionResult 
spCollideCircles(const spCircle* circleA, const spCircle* circleB)
{
    spCollisionResult result;
    result.colliding = spFalse;
    return result;
    /// get the circles
    //const spCircle* a = spShapeCastCircle(data.shape_a);
    //const spCircle* b = spShapeCastCircle(data.shape_b);

    ///// get the transforms
    //const spTransform* xfA = data.transform_a;
    //const spTransform* xfB = data.transform_b;

    ///// compute world centers
    //spVector cA = spMult(*xfA, a->center);
    //spVector cB = spMult(*xfB, b->center);

    ///// compute distance and distance^2 between the two centers
    //spVector distance = spSub(cB, cA);
    //spFloat distance2 = spDot(distance, distance);

    ///// get the combined radius and radius^2 of the two circle
    //spFloat radius    = a->radius + b->radius;
    //spFloat radius2   = radius * radius;

    ///// the circle arent colliding
    //if (distance2 > radius2)
    //{
    //    return spFalse;
    //}

    ///// compute point and normal, and penetration
    //spFloat penetration = radius - spsqrt(distance2);
    //spVector point  = spLerp(cA, cB, a->radius / radius);
    //spVector normal = spNormal(distance);

    ///// initialize the contact
    //initContact(contact, 0, 1, point, cA, cB, normal, penetration, data);

    ///// successful collision
    //return spTrue;
}

static spVector
extremalPointPoly(const spPolygon* poly, const spVector normal)
{
    spTransform* xf = &poly->base_class.body->xf;

    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count   = poly->count;

    spFloat maxProj = -SP_MAX_FLT;    /// largest projection
    spVector maxVec = spVectorZero(); /// most extreme point

    /// find the most extreme point along a direction
    for (spInt i = 0; i < count; ++i)
    {
        spEdge* edge = edges + i;
        spVector   v = spMult(*xf, edge->vertex);
        spFloat proj = spDot(v, normal);

        /// this projection is larger, save its info
        if (proj > maxProj)
        {
            maxProj = proj;
            maxVec = v;
        }
    }

    /// return the extremal vertex of the poly along a direction
    return maxVec;
}

static Edge
extremalEdgePoly(const spPolygon* poly, const spVector& normal)
{
    spTransform* xf = &poly->base_class.body->xf;

    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count   = poly->count;

    spFloat A = -SP_MAX_FLT; /// largest projection
    spFloat B = -SP_MAX_FLT; /// largest projection
    Edge e;

    /// find the most extreme point along a direction
    for (spInt i = 0; i < count; ++i)
    {
        spEdge* edge = edges + i;
        spVector   v = spMult(*xf, edge->vertex);
        spFloat proj = spDot(v, normal);

        if (B < proj)
        {
            if (i != 0)
            {
            e.a = e.b;
            A = B;
            }

            e.b = v;
            B = proj;
        }
        else if (A < proj)
        {
            e.a = v;
            A = proj;
        }
    }
    return e;
}

static struct spMinkowskiPoint
supportPoint(const struct SupportPointContext* context, const spVector normal)
{
    spSupportPoint pointA = context->supportPointA(context->shapeA, normal);
    spSupportPoint pointB = context->supportPointB(context->shapeB, spNegate(normal));

    return spMinkowskiPointConstruct(pointA.point, pointB.point);
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

static inline spVector
spClosestPointToOrigin(spVector t, spVector h)
{
    /// http://www.geometrictools.com/Documentation/DistancePointLine.pdf
    /// P = origin (0, 0).
    /// B = t
    spVector M = spSub(h, t);
    spFloat t0 = spClamp(spDot(M, spNegate(t))/spLengthSquared(M), 0.0f, 1.0f);
    return spNegate(spAdd(t, spMult(M, t0)));
}

static inline spFloat
spDistToOriginSq(spVector t, spVector h)
{
    /// t = vector tail
    /// h = vector head
    /// edge = sub(head, tail)
    return spLengthSquared(spClosestPointToOrigin(t, h));
}

static inline spFloat
spDistToOrigin(spVector t, spVector h)
{
    /// t = vector tail
    /// h = vector head
    /// edge = sub(head, tail)
    return spLength(spClosestPointToOrigin(t, h));
}

static spCollisionResult
clipEdges(const struct Edge* a, const struct Edge* b, const struct MinkowskiEdge* edge)
{
    spCollisionResult result = spCollisionResultConstruct();
    spVector normal = result.normal = edge->normal;

    /// distance of points along perp axis of the normal
    spFloat distAa = spCross(a->a, normal); spFloat distAb = spCross(a->b, normal);
    spFloat distBa = spCross(b->a, normal); spFloat distBb = spCross(b->b, normal);

    /// distance of edges a and b
    spFloat distA = distAb - distAa;
    spFloat distB = distBb - distBa;

    /// inverse distances used to normalize lerp ratio between [0:1]
    spFloat invDistA = distA ? 1.0f / distA : 0.0f;
    spFloat invDistB = distB ? 1.0f / distB : 0.0f;

    {
        /// get lerp ratios of the clipped points
        spFloat tA = spClamp((distBb - distAa) * invDistA, 0.f, 1.f);
        spFloat tB = spClamp((distAa - distBa) * invDistB, 0.f, 1.f);

        /// compute the points in world space
        spVector pointA = spLerp(a->a, a->b, tA);
        spVector pointB = spLerp(b->a, b->b, tB);

        /// compute the penetration to see if they are in contact
        spFloat penetration = -spDot(spSub(pointB, pointA), normal);
        if (penetration >= 0.0f)
        {
            result.colliding = spTrue;
            result.pointA[result.count] = pointA;
            result.pointB[result.count] = pointB;
            result.count++;
        }
    } {
        /// get lerp ratios of the clipped points
        spFloat tA = spClamp((distBa - distAa) * invDistA, 0.f, 1.f);
        spFloat tB = spClamp((distAb - distBa) * invDistB, 0.f, 1.f);

        /// compute the points in world space
        spVector pointA = spLerp(a->a, a->b, tA);
        spVector pointB = spLerp(b->a, b->b, tB);

        /// compute the penetration to see if they are in contact
        spFloat penetration = -spDot(spSub(pointB, pointA), normal);
        if (penetration >= 0.0f)
        {
            result.colliding = spTrue;
            result.pointA[result.count] = pointA;
            result.pointB[result.count] = pointB;
            result.count++;
        }
    }
    return result;
}

static MinkowskiEdge
EPA(const struct SupportPointContext* context, spMinkowskiPoint* m0, spMinkowskiPoint* m1, spMinkowskiPoint* m2)
{
    /// allocate space for the hulls
    spMinkowskiPoint  hull0[32] = { 0 };
    spMinkowskiPoint  hull1[32] = { 0 };
    spMinkowskiPoint* hull = NULL;
    spInt count = 3;

    /// initialize hull
    hull0[0] = *m0;
    hull0[1] = *m1;
    hull0[2] = *m2;
    hull = hull0;

    /// max epa iterations
    static const spInt max_iters = 29;

    /// iterate through EPA algorithm
    for (spInt iter = 0; iter < max_iters; ++iter)
    {
        spFloat min_dist = SP_MAX_FLT; /// min distance to an edge
        spInt ti = 0;                  /// tail index
        spInt hi = 0;                  /// head index

        for (spInt t = count-1, h = 0; h < count; t = h, h++)
        {
            /// find the closest edge to the origin
            spFloat d = spDistToOriginSq(hull[t].v, hull[h].v);
            if (d < min_dist)
            {
                min_dist = d;
                ti = t;
                hi = h;
            }
        }

        /// get the two minkowski points on the hull with the closest edge to the origin
        spMinkowskiPoint head = hull[hi];
        spMinkowskiPoint tail = hull[ti];

        /// get the new point on the hull
        spVector dir = spSkewT(spSub(head.v, tail.v));
        spMinkowskiPoint m = supportPoint(context, dir);

        /// check if the point it already in the hull
        for (spInt i = 0; i < count; ++i)
        {
            if (spEqual(hull[i].v, m.v))
            {
                /// the point is already on the hull, return the minkowski edge
                return MinkowskiEdgeConstruct(&head, &tail);
            }
        }

        /// rebuild the hull
        spMinkowskiPoint* hull_old;
        spMinkowskiPoint* hull_new;

        /// check if we are on an odd number iter
        spBool odd = iter & 1;
        hull_old = odd ? hull1 : hull0;
        hull_new = odd ? hull0 : hull1;

        count++;

        /// copy the hull over
        for (spInt n = 0, o = 0; n < count; ++n)
        {
            hull_new[n] = (n == hi) ? m : hull_old[o++];
        }
        hull = hull_new;
    }
    spAssert(false, "increase EPA iters, or remove poly's with large amounts of vertices\n");
}

static struct MinkowskiEdge 
GJK(const struct SupportPointContext* context)
{
    /// get the shape pointers
    const spShape* shapeA = context->shapeA;
    const spShape* shapeB = context->shapeB;

    /// get the transforms
    const spTransform* xfA = &shapeA->body->xf;
    const spTransform* xfB = &shapeB->body->xf;

    /// generate an initial axis direction for support points
    spVector cA = spMult(*xfA, spShapeGetCenter(shapeA));
    spVector cB = spMult(*xfB, spShapeGetCenter(shapeB));

    /// calculate normal directions for support points
    spVector normal = spSkew(spSub(cA, cB));
    spVector negate = spNegate(normal);

    /// calculate initial minkowski points
    /// TODO: cache points between frames
    spMinkowskiPoint m0 = supportPoint(context, normal);
    spMinkowskiPoint m1 = supportPoint(context, negate);

    /// make sure the origin is always to the left of the edge
    if (spOriginToRight(m0.v, m1.v)) 
    {
        spSwap(&m0.v, &m1.v);
        spSwap(&m0.a, &m1.a);
        spSwap(&m0.b, &m1.b);
    }

    static const spInt max_iters = 16;
    for (spInt i = 0; i < max_iters; ++i)
    {
        /// calculate a new direction for the next support point
        spVector dir = spSkew(spSub(m0.v, m1.v));

        /// expand the simplex by generating a new support point
        spMinkowskiPoint m2 = supportPoint(context, dir);

        /// check if the origin is inside of the 3-simplex or the new minkowski point is on origin
        if (spOriginToLeft(m1.v, m2.v) && spOriginToLeft(m2.v, m0.v) || spEqual(m2.v, spVectorZero()))
        {
            /// the origin is in the simplex, pass the 3-simplex to EPA and generate contact info
            return EPA(context, &m0, &m2, &m1);
        }
        else
        {
            /// check if this is the closest edge to the origin.
            if (spMax(spDot(m0.v, dir), spDot(m1.v, dir)) >= spDot(m2.v, dir))
            {
                return MinkowskiEdgeConstruct(&m1, &m0);
            }
            else
            {
                /// check which edge contains a closer point to the origin
                spBool m0isCloser = spDistToOriginSq(m0.v, m2.v) > spDistToOriginSq(m1.v, m2.v);

                /// remove the uneeded point from the simplex that is farther away
                m0isCloser ? m0 = m2 : m1 = m2;
            }
        }
    }

    /// GJK terminated without converging and without an intersection.
    return MinkowskiEdgeConstruct(&m1, &m0);
}

static spCollisionResult
spCollidePolygons(const spPolygon* polyA, const spPolygon* polyB)
{
    SupportPointFunc polySupport = (SupportPointFunc)extremalPointPoly;
    struct SupportPointContext context = { (spShape*)polyA, (spShape*)polyB, polySupport, polySupport };

    struct MinkowskiEdge minkowskiEdge = GJK(&context);

    if (minkowskiEdge.distance >= 0.0f)
    {
        spVector normal = minkowskiEdge.normal;
        spVector negate = spNegate(normal);

        Edge edgeA = extremalEdgePoly(polyA,  normal);
        Edge edgeB = extremalEdgePoly(polyB,  negate);

        return clipEdges(&edgeA, &edgeB, &minkowskiEdge);
    }
    else
    {
        spCollisionResult result;
    	result.colliding = spFalse;
    	return result;
    }
}

spCollisionMatrix 
_spCollisionMatrix()
{
    spCollisionMatrix matrix;

    matrix.CollideFunc[0][0] = (spCollisionFunc)spCollideCircles;
    matrix.CollideFunc[1][0] = (spCollisionFunc)spCollideCirclePolygon;
    matrix.CollideFunc[2][0] = 0;
    
    matrix.CollideFunc[0][1] = (spCollisionFunc)spCollidePolygonCircle;
    matrix.CollideFunc[1][1] = (spCollisionFunc)spCollidePolygons;
    matrix.CollideFunc[2][1] = 0;

    matrix.CollideFunc[0][2] = 0;
    matrix.CollideFunc[1][2] = 0;
    matrix.CollideFunc[2][2] = 0;

    return matrix;
}