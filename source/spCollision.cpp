

#include "spCollision.h"
#include "spContact.h"
#include "spDebugDraw.h"
#include <stdio.h>
#include "spBody.h"

#define logFloat(a) spLog("%.7f\n", a)

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
    spVector point;
    spFloat distance;
    spFloat t;
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
    edge.distance = distance;
    edge.normal = normal;
    edge.point = point;
    edge.head = *head;
    edge.tail = *tail;
    edge.t = t;

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

static void
addContact(spCollisionResult* result, const spVector pointA, const spVector pointB)
{
    spAssert(result->count <= 2, "cannot add more than 2 contacts!");

    result->colliding = spTrue;
	result->pointA[result->count] = pointA;
	result->pointB[result->count] = pointB;
	result->count++;
}

static void
invertContacts(spCollisionResult* result)
{
    for (spInt i = 0; i < result->count; ++i)
    {
        spVector tmp = result->pointA[i];
		result->pointA[i] = result->pointB[i];
		result->pointB[i] = tmp;
    }
    result->normal = spNegate(result->normal);
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

static spVector
extremalPointCircle(const spCircle* circle, const spVector normal)
{
    spTransform* xf = &circle->base_class.body->xf;
    return spMult(*xf, circle->center);
}

//static spVector
//extremalPointSegment(const spSegment* segment, const spVector normal)
//{
//}

static spVector
extremalPointPoly(const spPolygon* poly, const spVector normal)
{
    spTransform* xf = &poly->base_class.body->xf;

    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count   = poly->count;

    spFloat maxProj = -SP_MAX_FLT;
    spVector maxVec = spVectorZero();

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
            addContact(&result, pointA, pointB);
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
            addContact(&result, pointA, pointB);
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
                spLog("EPA %d\n", iter);
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
                spLog("GJK\n");
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

/// Collision functions

static spCollisionResult 
spCollideCircles(const spCircle* circleA, const spCircle* circleB)
{
    return spCollisionResultConstruct();
}

static spCollisionResult 
spCollidePolygonCircle(const spPolygon* poly, const spCircle* circle)
{
    struct SupportPointContext context = { 
        (spShape*)poly, 
        (spShape*)circle, 
        (SupportPointFunc)extremalPointPoly, 
        (SupportPointFunc)extremalPointCircle };

    struct MinkowskiEdge minkowskiEdge = GJK(&context);

    spCollisionResult result = spCollisionResultConstruct();

    /// check if they are potentially colliding
    if (minkowskiEdge.distance + circle->radius >= 0.0f)
    {
        /// get the two minkowski points on the edge
        spMinkowskiPoint* head = &minkowskiEdge.head;
        spMinkowskiPoint* tail = &minkowskiEdge.tail;

        /// get the lerp value and normal of the edge
        spFloat t = minkowskiEdge.t;
        spVector normal = result.normal = minkowskiEdge.normal;

        /// compute the two world space points of each body given the lerp value and edge vertices
        spVector pointA = spLerp(tail->a, head->a, t);
        spVector pointB = spLerp(tail->b, head->b, t);

        /// vertex/vertex collisions need their normal adjusted
        if (t == 1.0f || t == 0.0f)
        {
            /// compute the new normal
            normal = result.normal = spNormal(spSub(pointB, pointA));

            /// compute the distance between the two vertices
            spFloat distance = spDot(normal, minkowskiEdge.point);

            /// check if they are colliding
            if (distance + circle->radius < 0.0f)
        	{
                /// they are too far apart, they cant be colliding
                result.colliding = spFalse;
                return result;
        	}
        }

        /// compute the circles world space point since we test it as a point
        pointB = spAdd(pointB, spMult(circle->radius, spNegate(normal)));;

        /// add the contact to the collision result
        addContact(&result, pointA, pointB);
    }

    /// return the collision result
    return result;
}

static spCollisionResult 
spCollideCirclePolygon(const spCircle* circle, const spPolygon* poly)
{
    spCollisionResult result = spCollidePolygonCircle(poly, circle);
    invertContacts(&result);
    return result;
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

    return spCollisionResultConstruct();
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