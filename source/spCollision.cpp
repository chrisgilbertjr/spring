
#include "spCollision.h"
#include "spContact.h"
#include "spDebugDraw.h"
#include <stdio.h>
#include "spBody.h"

static inline void
initContact(spContact* contact, spInt index, spInt count, spVector point, spVector pointA, spVector pointB, spVector normal, spFloat pen, const spCollisionInput& data)
{
    const spMaterial* mA = &data.shape_a->material;
    const spMaterial* mB = &data.shape_b->material;

    contact->points[index].p = point;
    contact->points[index].pen = pen;
    contact->points[index].r_a = spSub(point, pointA);
    contact->points[index].r_b = spSub(point, pointB);

    contact->count = count;
    contact->normal = normal;
    contact->friction = spMaterialComputeFriction(mA, mB);
    contact->restitution = spMaterialComputeRestitution(mA, mB);
}

static inline void
shiftContactPoints(spContact* contact)
{
    contact->points[0] = contact->points[1];
}

static inline void
swapContactPoints(spContact* contact)
{
    spContactPoint tmp = contact->points[0];
    contact->points[0] = contact->points[1];
    contact->points[1] = tmp;
}

static inline spBool
withinSlop(spContact* contact, spInt index, spVector point, spFloat slop)
{
    return spAlmostEqual(contact->points[index].p, point, slop);
}

static inline void 
initContactPoint(spContact* contact, spInt index, spVector point, spVector pointA, spVector pointB)
{
}

static inline void
invertContact(spContact* contact)
{
    /// swap the normal and relative velocities
    spNegate(&contact->normal);
    spVector tmp = contact->points[0].r_a;
    contact->points[0].r_a = contact->points[0].r_b;
    contact->points[0].r_b = tmp;
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

static spBool
spCollideEdgeCircle(const spEdge* edge, const spCircle* circle)
{
    /// TODO:
    return spFalse;
}

static spBool 
spCollidePolygonCircle(spContact*& contact, const spCollisionInput& data)
{
    /// get the shapes
    const spShape* polyShape   = data.shape_a;
    const spShape* circleShape = data.shape_b;

    const spPolygon* poly   = spShapeCastPolygon(polyShape);
    const spCircle*  circle = spShapeCastCircle(circleShape);

    /// get the shapes transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

    /// get local centers
    spVector lcA = spShapeGetCenter(polyShape);
    spVector lcB = spShapeGetCenter(circleShape);

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

        /// the separation is > radius, they are not colliding
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

    /// collision normal and point
    spVector normal, point;
    spFloat penetration;

    /// the circles center is inside of the polygon
    /// TODO:
    if (maxSep < SP_FLT_EPSILON)
    {
        normal = spMult(xfA->q, edges[i0].normal);
        point = spAdd(cA, spMult(spNegate(normal), radius));
        penetration = radius;

        initContact(contact, 0, 1, point, cA, cB, normal, penetration, data);
        return spTrue;
    }

    /// compute contact penetration
    //penetration = radius - maxSep;

    /// compute voronoi regions
    spFloat voronoi0 = spDot(spSub(center, v0), spSub(v1, v0));
    spFloat voronoi1 = spDot(spSub(center, v1), spSub(v0, v1));
    spFloat pen2 = radius - maxSep;

    /// the point is to the left and in v0's voronoi region
    if (voronoi0 <= 0.0f)
    {
        spFloat distance2 = spDistanceSquared(center, v0);

        /// check if the point is inside of the circle
        if (distance2 > radius2) return spFalse;

        /// compute the point and normal
        penetration = radius - spsqrt(distance2);
        normal = spNormal(spMult(xfA->q, spSub(center, v0)));
        point  = spMult(*xfA, v0);
    }

    /// the point is to the right and in v1's voronoi region
    else if (voronoi1 <= 0.0f)
    {
        spFloat distance2 = spDistanceSquared(center, v1);

        /// check if the point is inside of the circle
        if (distance2 > radius2) return spFalse;

        penetration = radius - spsqrt(distance2);
        normal = spNormal(spMult(xfA->q, spSub(center, v1)));
        point  = spMult(*xfA, v1);
    }
    
    /// the point is in between v1 and v2, its on the edges voronoi region
    else
    {
        /// check if the point is inside of the circle
        if (spDot(spSub(center, v0), edges[i0].normal) > radius) return spFalse;

        penetration = radius - maxSep;
        normal = spNormal(spMult(xfA->q, edges[i0].normal));
        point  = spAdd(spMult(spNegate(normal), radius), data.shape_b->body->p);
    }


    /// initialize the contact and return a successful collision
    initContact(contact, 0, 1, point, cA, cB, normal, penetration, data);
    return spTrue;
}

static spBool 
spCollideCirclePolygon(spContact*& contact, const spCollisionInput& data)
{
    /// swap the collision data, and do the collision
    spBool result = spCollidePolygonCircle(contact, spCollisionInputSwap(data));

    /// swap the normal and relative velocities
    invertContact(contact);

    /// return the result
    return result;
}

static spBool 
spCollideCircleChain(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

static spBool 
spCollideChainCircle(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

static spBool 
spCollidePolygonChain(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

static spBool 
spCollideChainPolygon(spContact*& contact, const spCollisionInput& data)
{
    /// TODO:
    return spFalse;
}

static spBool 
spCollideCircles(spContact*& contact, const spCollisionInput& data)
{
    /// get the circles
    const spCircle* a = spShapeCastCircle(data.shape_a);
    const spCircle* b = spShapeCastCircle(data.shape_b);

    /// get the transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

    /// compute world centers
    spVector cA = spMult(*xfA, a->center);
    spVector cB = spMult(*xfB, b->center);

    /// compute distance and distance^2 between the two centers
    spVector distance = spSub(cB, cA);
    spFloat distance2 = spDot(distance, distance);

    /// get the combined radius and radius^2 of the two circle
    spFloat radius    = a->radius + b->radius;
    spFloat radius2   = radius * radius;

    /// the circle arent colliding
    if (distance2 > radius2)
    {
        return spFalse;
    }

    /// compute point and normal, and penetration
    spFloat penetration = radius - spsqrt(distance2);
    spVector point  = spLerp(cA, cB, a->radius / radius);
    spVector normal = spNormal(distance);

    /// initialize the contact
    initContact(contact, 0, 1, point, cA, cB, normal, penetration, data);

    /// successful collision
    return spTrue;
}

static spVector
spExtremalQuery(const spPolygon* poly, const spTransform* xf, const spVector& normal)
{
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

struct Edge
{
    spVector a;
    spVector b;
};

static Edge
spExtremalQueryEdge(const spPolygon* poly, const spTransform* xf, const spVector& normal)
{
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

static spMinkowskiPoint
spSupportPoint(const spPolygon* a, const spPolygon* b, const spTransform* xfa, const spTransform* xfb, const spVector& normal)
{
    /// get the extreme points
    spVector va = spExtremalQuery(a, xfa, normal);
    spVector vb = spExtremalQuery(b, xfb, spNegate(normal));

    /// create and return a minkowski point
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
spLerpRatio(spVector t, spVector h)
{
    /// http://www.geometrictools.com/Documentation/DistancePointLine.pdf
    spVector M = spSub(h, t);
    return spClamp(spDot(M, spNegate(t))/spLengthSquared(M), 0.0f, 1.0f);
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

static void
spEPAContactPoints(spContact*& contact, spMinkowskiPoint* head, spMinkowskiPoint* tail, const spCollisionInput& data)
{
    // get the shape pointers
    const spShape* shapeA = data.shape_a;
    const spShape* shapeB = data.shape_b;

    /// get the transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

    // get the polygons polygons
    spPolygon* a = spShapeCastPolygon(shapeA);
    spPolygon* b = spShapeCastPolygon(shapeB);

    /// get the points in world space
    spVector ba = spMult(*xfA, spShapeGetCenter(shapeA));
    spVector bb = spMult(*xfB, spShapeGetCenter(shapeB));

    /// get the vector of the two world points
    spVector v = spSub(head->v, tail->v);

    /// calculate the contact normal
    spVector normal = spNormal(spSkewT(v));
    spVector negate = spNegate(normal);
    spFloat  t = spLerpRatio(tail->v, head->v);
    spVector p = spLerp(tail->v, head->v, t);
    spFloat dist = spDot(normal, p);

    Edge e1 = spExtremalQueryEdge(a, xfA, normal);
    Edge e2 = spExtremalQueryEdge(b, xfB, negate);

    {
        spVector n = normal;

        spFloat de1a = spCross(e1.a, n);
        spFloat de1b = spCross(e1.b, n);
        spFloat de2a = spCross(e2.a, n);
        spFloat de2b = spCross(e2.b, n);

        spFloat e1_denom = 1.0f / (de1b - de1a);
        spFloat e2_denom = 1.0f / (de2b - de2a);

        spInt index = 0;
        spInt count = 1;

        static spBool swap = spFalse;

        {
            spVector pA = spLerp(e1.a, e1.b, spClamp((de2b - de1a) * e1_denom, 0.0f, 1.0f));
            spVector pB = spLerp(e2.a, e2.b, spClamp((de1a - de2a) * e2_denom, 0.0f, 1.0f));
            spFloat dist = spDot(spSub(pB, pA), n);
            if (dist <= 0.0f)
            {
                const spMaterial* mA = &data.shape_a->material;
                const spMaterial* mB = &data.shape_b->material;

    			contact->points[index].p = pB;
    			contact->points[index].pen = -dist;
    			contact->points[index].r_a = spSub(pA, shapeA->body->p);
    			contact->points[index].r_b = spSub(pB, shapeB->body->p);

    			contact->count = count;
    			contact->normal = normal;
    			contact->friction = spMaterialComputeFriction(mA, mB);
    			contact->restitution = spMaterialComputeRestitution(mA, mB);
                index++;
                count++;
            }
        }
        {
            spVector pA = spLerp(e1.a, e1.b, spClamp((de2a - de1a) * e1_denom, 0.0f, 1.0f));
            spVector pB = spLerp(e2.a, e2.b, spClamp((de1b - de2a) * e2_denom, 0.0f, 1.0f));
            spFloat dist = spDot(spSub(pB, pA), n);
            if (dist <= 0.0f)
            {
                const spMaterial* mA = &data.shape_a->material;
                const spMaterial* mB = &data.shape_b->material;

    			contact->points[index].p = pB;
    			contact->points[index].pen = -dist;
    			contact->points[index].r_a = spSub(pA, shapeA->body->p);
    			contact->points[index].r_b = spSub(pB, shapeB->body->p);

    			contact->count = count;
    			contact->normal = normal;
    			contact->friction = spMaterialComputeFriction(mA, mB);
    			contact->restitution = spMaterialComputeRestitution(mA, mB);
                //initContact(contact++, index++, count, pB, ba, bb, n, -dist, data);

                /// improves numeric stability of persisting contacts
                /// sometimes contacts cause something to drift to one side,
                /// this keeps them from drifting
                swap = swap ? spFalse : spTrue;
                if (swap) swapContactPoints(contact);
            }
        }
    }

}

static void
spEPAContactPoints2(spContact*& contact, spMinkowskiPoint* head, spMinkowskiPoint* tail, const spCollisionInput& data)
{
    // get the shape pointers
    const spShape* shapeA = data.shape_a;
    const spShape* shapeB = data.shape_b;

    /// get the transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

    // get the polygons polygons
    spPolygon* a = spShapeCastPolygon(shapeA);
    spPolygon* b = spShapeCastPolygon(shapeB);

    /// compute the lerp t bettwen the two minkowski points
    spFloat t = spLerpRatio(tail->v, head->v);
    spFloat pen = spDistToOrigin(tail->v, head->v);

    /// get the points in world space
    spVector wa = spLerp(tail->a, head->a, t); 
    spVector wb = spLerp(tail->b, head->b, t);
    spVector ba = spMult(*xfA, spShapeGetCenter(shapeA));
    spVector bb = spMult(*xfB, spShapeGetCenter(shapeB));

    /// get the vector of the two world points
    spVector v = spSub(head->v, tail->v);

    /// calculate the contact normal
    spVector normal = spNormal(spSkewT(v));

    spInt index = 0;
    spInt count = 1;

    if (contact->count == 2)
    {
        shiftContactPoints(contact);
    }
    else if (contact->count == 1)
    {
        if (!withinSlop(contact, 0, wa, 0.05f))
        {
            index = 1; 
            count = 2;
        }
    }

    initContact(contact, index, count, wa, ba, bb, normal, pen, data);
}

static void
spEPA(spContact*& contact, spMinkowskiPoint* m0, spMinkowskiPoint* m1, spMinkowskiPoint* m2, const spCollisionInput& data)
{
    /// cast the shapes into polys
    const spPolygon* a = spShapeCastPolygon(data.shape_a);
    const spPolygon* b = spShapeCastPolygon(data.shape_b);

    /// get the transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

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
        spInt ti = 0;                  /// tail index
        spInt hi = 0;                  /// head index
        spFloat min_dist = SP_MAX_FLT; /// min distance to an edge

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
        spMinkowskiPoint m = spSupportPoint(a, b, xfA, xfB, dir);

        /// check if the point it already in the hull
        for (spInt i = 0; i < count; ++i)
        {
            if (spEqual(hull[i].v, m.v))
            {
                /// the point is already on the hull, init the contact points
                spEPAContactPoints(contact, &head, &tail, data);
                return;
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
}

static spBool 
spGJK(spContact*& contact, const spCollisionInput& data)
{
    /// TODO: update the algorithm to pass in function pointers for
    ///       support points so it can support any convex shape

    /// get the shape pointers
    const spShape* aShape = data.shape_a;
    const spShape* bShape = data.shape_b;

    /// cast them to actual shapes
    const spPolygon* a  = spShapeCastPolygon(aShape);
    const spPolygon* b  = spShapeCastPolygon(bShape);

    /// get the transforms
    const spTransform* xfA = data.transform_a;
    const spTransform* xfB = data.transform_b;

    /// generate an initial axis direction for support points
    spVector cA = spMult(*xfA, spShapeGetCenter(aShape));
    spVector cB = spMult(*xfB, spShapeGetCenter(bShape));
    spVector dir = spSkew(spSub(cA, cB));

    /// calculate initial minkowski points
    spMinkowskiPoint m0 = spSupportPoint(a, b, xfA, xfB, dir);
    spMinkowskiPoint m1 = spSupportPoint(a, b, xfA, xfB, spNegate(dir));

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
        spMinkowskiPoint m2 = spSupportPoint(a, b, xfA, xfB, dir);

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
                return spFalse;
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
    return spFalse;
}

spBool 
spCollidePolygons(spContact*& contact, const spCollisionInput& data)
{
    return spGJK(contact, data);
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