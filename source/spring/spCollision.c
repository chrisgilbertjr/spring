
#include "spCollision.h"
#include "spSegment.h"
#include "spPolygon.h"
#include "spCircle.h"
#include "spBody.h"

/// typedef structs for convenience
typedef struct SupportPointContext SupportPointContext;
typedef struct spMinkowskiPoint spMinkowskiPoint;
typedef struct MinkowskiEdge MinkowskiEdge;
typedef struct ClosestPoints ClosestPoints;
typedef struct Edge Edge;

/// an edge of two points
struct Edge
{
    spVector a, b; ///< edge points
};

/// a point on the minkowski difference of two shapes
struct spMinkowskiPoint
{
    spVector a; /// a point on shape a in world coords
    spVector b; /// a point on shape b in world coords
    spVector v; /// vector b - a
};

struct MinkowskiEdge
{
    spMinkowskiPoint head; ///< head point
    spMinkowskiPoint tail; ///< tail point
    spVector normal;       ///< collision normal of the edge
    spVector point;        ///< interpolated minkowski points
    spFloat distance;      ///< distance of the two world points (contact separation)
    spFloat t;             ///< lerp ratio of the two world points
};

/// closest points computed from a minkowski edge
struct ClosestPoints
{
    spVector a, b; ///< closest points in world space
};

/// defines support functions for a general purpose GJK/EPA
struct SupportPointContext
{
    spShape* shapeA;                ///< the first shape
    spShape* shapeB;                ///< the second shape
    SupportPointFunc supportPointA; ///< the first  shapes support point function
    SupportPointFunc supportPointB; ///< the second shapes support point function
};

static INLINE spFloat
spLerpRatio(spVector t, spVector h)
{
    /// http://www.geometrictools.com/Documentation/DistancePointLine.pdf
    /// lerp ratio of the origin onto the vector h - t
    spVector M = spSubVecs(h, t);
    return spClamp(spDot(M, spNegative(t))/spLengthSquared(M), 0.0f, 1.0f);
}

static INLINE spBool
spOriginToLeft(spVector a, spVector b)
{
    return (a.x - b.x) * (a.y + b.y) < (a.y - b.y) * (a.x + b.x);
}

static INLINE spBool
spOriginToRight(spVector a, spVector b)
{
    return (a.x - b.x) * (a.y + b.y) > (a.y - b.y) * (a.x + b.x);
}

static INLINE spVector
spClosestPointToOrigin(spVector t, spVector h)
{
    /// http://www.geometrictools.com/Documentation/DistancePointLine.pdf
    /// P = origin (0, 0).
    /// B = t
    spVector M = spSubVecs(h, t);
    spFloat t0 = spClamp(spDot(M, spNegative(t))/spLengthSquared(M), 0.0f, 1.0f);
    return spNegative(spAddVecs(t, spMultVecFlt(M, t0)));
}

static INLINE spFloat
spDistToOriginSq(spVector t, spVector h)
{
    /// t = vector tail
    /// h = vector head
    /// edge = sub(head, tail)
    return spLengthSquared(spClosestPointToOrigin(t, h));
}

static INLINE spFloat
spDistToOrigin(spVector t, spVector h)
{
    /// t = vector tail
    /// h = vector head
    /// edge = sub(head, tail)
    return spLength(spClosestPointToOrigin(t, h));
}

static INLINE struct MinkowskiEdge
MinkowskiEdgeConstruct(spMinkowskiPoint* head, spMinkowskiPoint* tail)
{
    NULLCHECK(head); NULLCHECK(tail);
    /// get the closest minkowski point
    spFloat t = spLerpRatio(tail->v, head->v);
    spVector point = spLerpVec(tail->v, head->v, t);

    /// calculate the contact normal and penetration distance
    spVector delta   = spSubVecs(head->v, tail->v);
    spVector normal  = spNormal(spSkewT(delta));
    spFloat distance = spDot(normal, point);

    /// init the minkowski edge
    MinkowskiEdge edge;
    edge.distance = distance;
    edge.normal = normal;
    edge.point = point;
    edge.head = *head;
    edge.tail = *tail;
    edge.t = t;

    return edge;
}

static INLINE struct ClosestPoints
MinkowskiEdgeComputePoints(MinkowskiEdge* edge)
{
    NULLCHECK(edge);
    /// compute the two world space points of each body given the lerp value and edge vertices
    spVector pointA = spLerpVec(edge->tail.a, edge->head.a, edge->t);
    spVector pointB = spLerpVec(edge->tail.b, edge->head.b, edge->t);

    ClosestPoints points;
    points.a = pointA;
    points.b = pointB;

    return points;
};

static INLINE struct spCollisionResult
spCollisionResultConstruct()
{
    /// construct a new collision result
    spCollisionResult result;
    result.colliding = spFalse;
    result.pointA[0] = spVectorZero();
    result.pointB[0] = spVectorZero();
    result.pointA[1] = spVectorZero();
    result.pointB[1] = spVectorZero();
    result.normal    = spVectorZero();
    result.count     = 0;

    return result;
}

spMinkowskiPoint 
spMinkowskiPointConstruct(spVector a, spVector b)
{
    /// construct a new minkowski point
    spMinkowskiPoint m;
    m.v = spSubVecs(a, b);
    m.a = a;
    m.b = b;

    return m;
}

static void
addContact(spCollisionResult* result, const spVector pointA, const spVector pointB)
{
    NULLCHECK(result);
    spAssert(result->count <= 2, "cannot add more than 2 contacts!");

    /// add the contact to the collision result
    result->colliding = spTrue;
	result->pointA[result->count] = pointA;
	result->pointB[result->count] = pointB;
	result->count++;
}

static void
invertContacts(spCollisionResult* result)
{
    NULLCHECK(result);
    /// swap the contact info
    for (spInt i = 0; i < result->count; ++i)
    {
        spVector tmp = result->pointA[i];
		result->pointA[i] = result->pointB[i];
		result->pointB[i] = tmp;
    }
    result->normal = spNegative(result->normal);
}

static spVector
extremalPointCircle(const spCircle* circle, const spVector normal)
{
    NULLCHECK(circle);
    /// circles are tested as points with a radius, return the world center
    return spMultXformVec(circle->shape.body->xf, circle->center);
}

static spVector
extremalPointSegment(const spSegment* segment, const spVector normal)
{
    NULLCHECK(segment);
    spTransform* xf = &segment->shape.body->xf;
    
    spVector pointA = spMultXformVec(*xf, segment->pointA);
    spVector pointB = spMultXformVec(*xf, segment->pointB);

    return spDot(pointA, normal) > spDot(pointB, normal) ? pointA : pointB;
}

static Edge
extremalEdgeSegment(const spSegment* segment, const spVector normal)
{
    NULLCHECK(segment);
    spTransform* xf = &segment->shape.body->xf;
    
    spVector pointA = spMultXformVec(*xf, segment->pointA);
    spVector pointB = spMultXformVec(*xf, segment->pointB);

    Edge edge;

    if (spDot(segment->normal, normal) > 0.0f)
    {
        edge.a = pointB;
        edge.b = pointA;
    }
    else
    {
        edge.a = pointA;
        edge.b = pointB;
    }

    return edge;
}

static spInt
extremalIndexPoly(const spPolygon* poly, const spVector normal)
{
    NULLCHECK(poly);
    spTransform* xf = &poly->shape.body->xf;

    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count   = poly->count;

    spFloat maxProj = -SP_MAX_FLT;
    spInt index = 0;

    /// find the most extreme point along a direction
    for (spInt i = 0; i < count; ++i)
    {
        spEdge*    e = edges + i;
        spVector   v = spMultXformVec(*xf, e->vertex);
        spFloat proj = spDot(v, normal);

        /// this projection is larger, save its info
        if (proj > maxProj)
        {
            maxProj = proj;
            index = i;
        }
    }

    /// return the extremal vertex of the poly along a direction
    return index;
}

static spVector
extremalPointPoly(const spPolygon* poly, const spVector normal)
{
    NULLCHECK(poly);
    spTransform* xf = &poly->shape.body->xf;

    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count   = poly->count;

    spFloat maxProj = -SP_MAX_FLT;
    spVector maxVec = spVectorZero();

    /// find the most extreme point along a direction
    for (spInt i = 0; i < count; ++i)
    {
        spEdge*    e = edges + i;
        spVector   v = spMultXformVec(*xf, e->vertex);
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
extremalEdgePoly(const spPolygon* poly, const spVector normal)
{
    NULLCHECK(poly);
    spTransform* xf = &poly->shape.body->xf;

    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt   count = poly->count;

    /// get the edge vertex indices
    spInt index1 = extremalIndexPoly(poly, normal);
    spInt index2 = index1 == 0 ? count-1 : index1-1;
    spInt index0 = index1 == count-1 ? 0 : index1+1;

    /// get the normals rotated in world space
    spVector normal1 = spNormal(spMultRotVec(xf->q, edges[index1].normal));
    spVector normal2 = spNormal(spMultRotVec(xf->q, edges[index2].normal));

    Edge edge;
    if (spDot(normal, normal1) > spDot(normal, normal2))
    {
        edge.a = spMultXformVec(*xf, edges[index1].vertex);
        edge.b = spMultXformVec(*xf, edges[index0].vertex);
    }
    else
    {
        edge.a = spMultXformVec(*xf, edges[index2].vertex);
        edge.b = spMultXformVec(*xf, edges[index1].vertex);
    }

    return edge;
}

static struct spMinkowskiPoint
supportPoint(const struct SupportPointContext* context, const spVector normal)
{
    NULLCHECK(context);
    spVector pointA = context->supportPointA(context->shapeA, normal);
    spVector pointB = context->supportPointB(context->shapeB, spNegative(normal));

    return spMinkowskiPointConstruct(pointA, pointB);
}

/// contact point helper functions

static void
vertexVertexCorrection(MinkowskiEdge* edge, struct ClosestPoints points)
{
    NULLCHECK(edge);
    /// compute the new normal
	edge->normal = spNormal(spSubVecs(points.b, points.a));

    /// compute the distance between the two vertices
	edge->distance = spDot(edge->normal, edge->point);
}

static spBool
edgesOverlap(const struct Edge* a, const struct Edge* b, spVector normal)
{
    NULLCHECK(a); NULLCHECK(b);
    /// get the tangents of the edges given a normal
    spVector tangentA = spSkew(normal);
    spVector tangentB = spNegative(tangentA);

    /// compute the distance in tangentA's and tangentB's
    spFloat minDistA = spMin(spDot(a->a, tangentA), spDot(a->b, tangentA));
    spFloat minDistB = spMin(spDot(a->a, tangentB), spDot(a->b, tangentB));


    /// check if the edges overlap in the normal direction by checking if both points are > or < both tangents
    return (spDot(b->a, tangentA) > minDistA && spDot(b->b, tangentA) > minDistA) ||
           (spDot(b->a, tangentB) > minDistB && spDot(b->b, tangentB) > minDistB);
}

static spCollisionResult
clipEdges(const struct Edge* a, const struct Edge* b, const struct MinkowskiEdge* edge, spFloat radiusA, spFloat radiusB)
{
    NULLCHECK(a); NULLCHECK(b); NULLCHECK(edge);
    spCollisionResult result = spCollisionResultConstruct();
    spVector normal = result.normal = edge->normal;

    /// distance of points along perp axis of the normal
    spFloat distAa = spCrossVecs(a->a, normal); spFloat distAb = spCrossVecs(a->b, normal);
    spFloat distBa = spCrossVecs(b->a, normal); spFloat distBb = spCrossVecs(b->b, normal);

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

        /// compute the points in world space, include their radius
        spVector pointA = spAddVecs(spMultVecFlt(normal,  radiusA), spLerpVec(a->a, a->b, tA));
        spVector pointB = spAddVecs(spMultVecFlt(normal, -radiusB), spLerpVec(b->a, b->b, tB));

        /// compute the penetration to see if they are in contact
        spFloat penetration = -spDot(spSubVecs(pointB, pointA), normal);
        if (penetration > 0.0f)
        {
            addContact(&result, pointA, pointB);
        }
    } {
        /// get lerp ratios of the clipped points
        spFloat tA = spClamp((distBa - distAa) * invDistA, 0.f, 1.f);
        spFloat tB = spClamp((distAb - distBa) * invDistB, 0.f, 1.f);

        /// compute the points in world space, include their radius
        spVector pointA = spAddVecs(spMultVecFlt(normal,  radiusA), spLerpVec(a->a, a->b, tA));
        spVector pointB = spAddVecs(spMultVecFlt(normal, -radiusB), spLerpVec(b->a, b->b, tB));

        /// compute the penetration to see if they are in contact
        spFloat penetration = -spDot(spSubVecs(pointB, pointA), normal);
        if (penetration >= 0.0f)
        {
            addContact(&result, pointA, pointB);
        }
    }

    return result;
}

/// EPA/GJK

static MinkowskiEdge
EPA(const struct SupportPointContext* context, spMinkowskiPoint* m0, spMinkowskiPoint* m1, spMinkowskiPoint* m2)
{
    NULLCHECK(context); NULLCHECK(m0); NULLCHECK(m1); NULLCHECK(m2);
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
        spVector dir = spSkewT(spSubVecs(head.v, tail.v));
        spMinkowskiPoint m = supportPoint(context, dir);

        /// check if the point is already in the hull
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
    spWarning(spFalse, "increase EPA iters, or remove poly's with large amounts of vertices\n");
    return MinkowskiEdgeConstruct(hull+0, hull+1);
}

static struct MinkowskiEdge 
GJK(const struct SupportPointContext* context)
{
    NULLCHECK(context);
    /// get the shape pointers
    const spShape* shapeA = context->shapeA;
    const spShape* shapeB = context->shapeB;

    /// get the transforms
    const spTransform* xfA = &shapeA->body->xf;
    const spTransform* xfB = &shapeB->body->xf;

    /// generate an initial axis direction for support points
    spVector cA = spMultXformVec(*xfA, spShapeGetCOM(shapeA));
    spVector cB = spMultXformVec(*xfB, spShapeGetCOM(shapeB));

    /// calculate normal directions for support points
    spVector normal = spSkew(spSubVecs(cA, cB));
    spVector negate = spNegative(normal);

    /// calculate initial minkowski points
    /// TODO: cache points between frames
    spMinkowskiPoint m0 = supportPoint(context, normal);
    spMinkowskiPoint m1 = supportPoint(context, negate);

    /// make sure the origin is always to the left of the edge
    if (spOriginToRight(m0.v, m1.v)) 
    {
        spvSwap(&m0.v, &m1.v);
        spvSwap(&m0.a, &m1.a);
        spvSwap(&m0.b, &m1.b);
    }

    static const spInt max_iters = 16;
    for (spInt i = 0; i < max_iters; ++i)
    {
        /// calculate a new direction for the next support point
        spVector dir = spSkew(spSubVecs(m0.v, m1.v));

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
                if (m0isCloser)
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
    return MinkowskiEdgeConstruct(&m1, &m0);
}

/// Collision functions

static spCollisionResult 
CircleToCircle(const spCircle* circleA, const spCircle* circleB)
{
    NULLCHECK(circleA); NULLCHECK(circleB);
    spCollisionResult result = spCollisionResultConstruct();

    /// get the shapes transforms
    spTransform* xfA = &circleA->shape.body->xf;
    spTransform* xfB = &circleB->shape.body->xf;

    /// compute the centers in world space, and difference between the two vectors
    spVector centerA = spMultXformVec(*xfA, circleA->center);
    spVector centerB = spMultXformVec(*xfB, circleB->center);
    spVector delta = spSubVecs(centerB, centerA);

    /// get the combined radius of the circles, and compute the distance between them
    spFloat radiusA = circleA->radius;
    spFloat radiusB = circleB->radius;
    spFloat radius = radiusA + radiusB;
    spFloat distance2 = spLengthSquared(delta);

    /// if they overlap, generate contact info
    if (distance2 < radius * radius)
    {
        /// calculate the penetration, and collision normal
        spFloat pen = spsqrt(distance2);
        spVector normal = result.normal = pen != 0.0f ? spMultVecFlt(delta, 1.0f/pen) : spVectorConstruct(0.0f, 1.0f);

        /// compute the contact points
        spVector pointA = spAddVecs(centerA, spMultVecFlt(normal,  radiusA));
        spVector pointB = spAddVecs(centerB, spMultVecFlt(normal, -radiusB));

        /// add the contact
        addContact(&result, pointA, pointB);
    }
    return result;
}

static spCollisionResult 
PolygonToCircle(const spPolygon* poly, const spCircle* circle)
{
    NULLCHECK(poly); NULLCHECK(circle);
    struct SupportPointContext context = { 
        (spShape*)poly, 
        (spShape*)circle, 
        (SupportPointFunc)extremalPointPoly, 
        (SupportPointFunc)extremalPointCircle };

    struct MinkowskiEdge mEdge = GJK(&context);

    spCollisionResult result = spCollisionResultConstruct();

    ClosestPoints points = MinkowskiEdgeComputePoints(&mEdge);

    /// vertex/vertex collisions need their normal adjusted
    if (mEdge.t == 1.0f || mEdge.t == 0.0f)
    {    
        vertexVertexCorrection(&mEdge, points);
    }

    /// check if they are potentially colliding
    if (mEdge.distance + circle->radius + poly->radius >= 0.0f)
    {
        /// get the contact normal
        spVector normal = result.normal = mEdge.normal;

        /// compute the contact points
        spVector pointA = spAddVecs(points.a, spMultFltVec(poly->radius, normal));
        spVector pointB = spAddVecs(points.b, spMultFltVec(circle->radius, spNegative(normal)));;

        /// add the contact to the collision result
        addContact(&result, pointA, pointB);
    }

    /// return the collision result
    return result;
}

static spCollisionResult 
CircleToPolygon(const spCircle* circle, const spPolygon* poly)
{
    NULLCHECK(circle); NULLCHECK(poly);
    spCollisionResult result = PolygonToCircle(poly, circle);
    invertContacts(&result);
    return result;
}

static spCollisionResult
PolygonToPolygon2(const spPolygon* polyA, const spPolygon* polyB)
{
    NULLCHECK(polyA); NULLCHECK(polyB);
    struct SupportPointContext context = { 
        (spShape*)polyA, 
        (spShape*)polyB, 
        (SupportPointFunc)extremalPointPoly,
        (SupportPointFunc)extremalPointPoly };

    struct MinkowskiEdge mEdge = GJK(&context);

    /// check if they are are collising
    if (mEdge.distance + polyA->radius + polyB->radius >= 0.0f)
    {
        /// get the two normal directions
        spVector normal = mEdge.normal;
        spVector negate = spNegative(normal);

        /// compute the extreme edges in the normals directions
        Edge edgeA = extremalEdgePoly(polyA,  normal);
        Edge edgeB = extremalEdgePoly(polyB,  negate);

        /// clip edges to get contact points
        return clipEdges(&edgeA, &edgeB, &mEdge, polyA->radius, polyB->radius);
    }

    /// no collision occured
    else
    {
        return spCollisionResultConstruct();
    }
}

static spCollisionResult
PolygonToPolygon(const spPolygon* polyA, const spPolygon* polyB)
{
    NULLCHECK(polyA); NULLCHECK(polyB);
    struct SupportPointContext context = { 
        (spShape*)polyA, 
        (spShape*)polyB, 
        (SupportPointFunc)extremalPointPoly,
        (SupportPointFunc)extremalPointPoly };

    struct MinkowskiEdge mEdge = GJK(&context);

    /// check if they are are collising
    if (mEdge.distance + polyA->radius + polyB->radius >= 0.0f)
    {
        /// get the two normal directions
        /// bias the normal slightly so we dont get swapping edge points due to floating point error (what a HEADACHE!)
        spVector normal = spNormal(spAddVecs(mEdge.normal, spMultVecFlt(spVectorConstruct(0.0f, 1.0f), 1e-5f)));
        spVector negate = spNegative(normal);

        /// compute the extreme edges in the normals directions
        Edge edgeA = extremalEdgePoly(polyA,  normal);
        Edge edgeB = extremalEdgePoly(polyB,  negate);

        /// clip edges to get contact points
        return clipEdges(&edgeA, &edgeB, &mEdge, polyA->radius, polyB->radius);
    }

    /// no collision occured
    else
    {
        return spCollisionResultConstruct();
    }
}


static spCollisionResult
SegmentToCircle(const spSegment* segment, const spCircle* circle)
{
    NULLCHECK(segment); NULLCHECK(circle);
    struct SupportPointContext context = { 
        (spShape*)segment, 
        (spShape*)circle, 
        (SupportPointFunc)extremalPointSegment, 
        (SupportPointFunc)extremalPointCircle };

    struct MinkowskiEdge mEdge = GJK(&context);

    spCollisionResult result = spCollisionResultConstruct();

    /// compute the two world space points of each body given the lerp value and edge vertices
    ClosestPoints points = MinkowskiEdgeComputePoints(&mEdge);

    /// vertex/vertex collisions need their normals adjusted
    if (mEdge.t == 1.0f || mEdge.t == 0.0f)
    {
        vertexVertexCorrection(&mEdge, points);
    }

    /// check if they are potentially colliding
    if (mEdge.distance + circle->radius + segment->radius >= 0.0f)
    {
        /// get the contact normal
        spVector normal = result.normal = mEdge.normal;
        spVector negate = spNegative(normal);

        /// compute the circles world space point since we test it as a point
        spVector pointA = spAddVecs(points.a, spMultFltVec(segment->radius, normal));
        spVector pointB = spAddVecs(points.b, spMultFltVec(circle->radius,  negate));

        /// add the contact to the collision result
        addContact(&result, pointA, pointB);
    }

    /// return the collision result
    return result;
}

static spCollisionResult
CircleToSegment(const spCircle* circle, const spSegment* segment)
{
    NULLCHECK(circle); NULLCHECK(segment);
    /// collide the shapes and swap the contacts
    spCollisionResult result = SegmentToCircle(segment, circle);
    invertContacts(&result);
    return result;
}

static spCollisionResult
PolygonToSegment(const spPolygon* poly, const spSegment* segment)
{
    NULLCHECK(poly); NULLCHECK(segment);
    struct SupportPointContext context = { 
        (spShape*)poly, 
        (spShape*)segment, 
        (SupportPointFunc)extremalPointPoly,
        (SupportPointFunc)extremalPointSegment };

    struct MinkowskiEdge mEdge = GJK(&context);

    if (mEdge.distance + segment->radius + poly->radius >= 0.0f)
    {
        spVector normal = mEdge.normal;
        spVector negate = spNegative(normal);

        ClosestPoints points = MinkowskiEdgeComputePoints(&mEdge);
        if (spAlmostEqualVecs(points.b, spMultXformVec(segment->shape.body->xf, segment->pointA)) || /// 1e-2
            spAlmostEqualVecs(points.b, spMultXformVec(segment->shape.body->xf, segment->pointB)))
        {
            spCollisionResult result = spCollisionResultConstruct();
            vertexVertexCorrection(&mEdge, points);
            if (mEdge.distance + segment->radius + poly->radius >= 0.0f)
            {
                result.normal = mEdge.normal;
                normal = mEdge.normal;
                negate = spNegative(mEdge.normal);

                spVector pointA = spAddVecs(points.a, spMultFltVec(poly->radius, normal));
                spVector pointB = spAddVecs(points.b, spMultFltVec(segment->radius, negate));

                /// add the contact to the collision result
                addContact(&result, pointA, pointB);
            }
            return result;
        }

        /// compute the world space edges in a normal direction
        Edge edgeA = extremalEdgePoly(poly, normal);
        Edge edgeB = extremalEdgeSegment(segment, negate);
        return clipEdges(&edgeA, &edgeB, &mEdge, poly->radius, segment->radius);
    }

    /// there is no collision
    else
    {
        return spCollisionResultConstruct();
    }
}

static spCollisionResult
SegmentToPolygon(const spSegment* segment, const spPolygon* poly)
{
    NULLCHECK(segment); NULLCHECK(poly);
    /// collide the shapes and swap the contact info
    spCollisionResult result = PolygonToSegment(poly, segment);
    invertContacts(&result);
    return result;
}

static spCollisionResult
SegmentToSegment(const spSegment* segmentA, const spSegment* segmentB)
{
    NULLCHECK(segmentA); NULLCHECK(segmentB);
    /// segments cannot collide
    return spCollisionResultConstruct();
}

/// collision functions
spCollisionFunc CollideFunc[SP_SHAPE_COUNT][SP_SHAPE_COUNT] = 
{
    (spCollisionFunc)CircleToCircle,
    (spCollisionFunc)CircleToPolygon,
    (spCollisionFunc)CircleToSegment,
    
    (spCollisionFunc)PolygonToCircle,
    (spCollisionFunc)PolygonToPolygon,
    (spCollisionFunc)PolygonToSegment,

    (spCollisionFunc)SegmentToCircle,
    (spCollisionFunc)SegmentToPolygon,
    (spCollisionFunc)SegmentToSegment
};