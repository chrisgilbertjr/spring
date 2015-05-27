
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
    spNegate(&contact->normal);
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
        spVector point = spAdd(spMult(normal, rb), cb);
        DRAW_POINT(point, 0.0f, 0.0f, 1.0f);

        contact->count = 1;
        contact->normal = normal;
        contact->friction = spMaterialComputeFriction(ma, mb);
        contact->restitution = spMaterialComputeRestitution(ma, mb);
        contact->points[0].r_a = spSub(point, ca);
        contact->points[0].r_b = spSub(point, cb);
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
    spVector point = spMult(spAdd(world_a, world_b), 0.5f);

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

spVector
spExtremalQuery(const spPolygon* poly, const spTransform* xf, const spVector& normal)
{
    /// poly edges and count
    spEdge* edges = poly->edges;
    spInt count = poly->count;

    spFloat mproj = -SP_MAX_FLT; /// largest dot projection
    spVector mv = spVector(SP_MIN_FLT, SP_MIN_FLT); /// extreme vector

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
            mv = v;
        }
    }

    /// return the extremal point of the poly along a direction
    return mv;
}

spVector
spSupportPoint(const spPolygon* a, const spPolygon* b, const spTransform* xfa, const spTransform* xfb, const spVector& normal)
{
    spVector va = spExtremalQuery(a, xfa, normal);
    spVector vb = spExtremalQuery(b, xfb, spNegate(normal));
    return spSub(va, vb);
}

enum spSide
{
    spLeft   = -1,
    spLinear =  0,
    spRight  =  1
};

static inline spFloat
spWhichSide(const spVector* a, const spVector* b, const spVector* p)
{
    return ((b->x-a->x) * (p->y-a->y)) - ((b->y-a->y) * (p->x-a->x));
}

spFloat 
spGetOriginSide(spVector head, spVector tail)
{
    /// left  < 0
    /// on   == 0
    /// right > 0
    return ((tail.x-head.x) * (-head.y)) - ((tail.y-head.y) * (-head.x));
}

spBool spOriginInTriangle(spVector a, spVector b, spVector c)
{
    spDebugDrawTriangle(a, b, c, spGreen(0.1f));
    spDebugDrawPoint(a, spYellow(1.0f));
    spDebugDrawPoint(b, spYellow(0.6f));
    spDebugDrawPoint(c, spYellow(0.3f));

    spVector pa = spLerp(b, a, .5f);
    spVector pb = spLerp(c, b, .5f);
    spVector pc = spLerp(a, c, .5f);

    spVector na = spSkew(spSub(b, a));
    spVector nb = spSkew(spSub(c, b));
    spVector nc = spSkew(spSub(a, c));

    spNormalize(&na);
    spNormalize(&nb);
    spNormalize(&nc);

    spDebugDrawLine(pa, spAdd(pa, na), spPurple(1.0f));
    spDebugDrawLine(pb, spAdd(pb, nb), spPurple(1.0f));
    spDebugDrawLine(pc, spAdd(pc, nc), spPurple(1.0f));

    return spWhichSide(&a, &b, &spVectorZero()) >= 0.0f &&
           spWhichSide(&b, &c, &spVectorZero()) >= 0.0f &&
           spWhichSide(&c, &a, &spVectorZero()) >= 0.0f;
}

static spBool 
spGJK(spClosestPoints* points, spContact*& contact, const spCollisionInput& data)
{
    const spTransform* xfa = data.transform_a;
    const spTransform* xfb = data.transform_b;
    const spPolygon*   pa  = (spPolygon*)data.shape_a;
    const spPolygon*   pb  = (spPolygon*)data.shape_b;
    const spBound*     ba  = &pa->base_class.bound;
    const spBound*     bb  = &pb->base_class.bound;

    /// calculate a general axis direction for GJK to start with
    spVector bca = spMult(*xfa, spBoundGetCenter(ba));
    spVector bcb = spMult(*xfb, spBoundGetCenter(bb));
    spVector dir = spSkew(spSub(bca, bcb));

    /// calculate initial minkowski points
    spVector m0 = spSupportPoint(pa, pb, xfa, xfb, dir);
    spVector m1 = spSupportPoint(pa, pb, xfa, xfb, spNegate(dir));

#ifdef SP_DEBUG_DRAW
    /// draw all minkowski points for debugging purposes
    for (spInt i = 0; i < pb->count; ++i)
    {
        spEdge*  eb = pb->edges;
        spVector vb = spMult(*xfb, eb[i].vertex);
        for (spInt j = 0; j < pa->count; ++j)
        {
            spEdge*  ea = pb->edges;
            spVector va = spMult(*xfa, ea[j].vertex);

            spVector p = spSub(va, vb);
            spDebugDrawPoint(p, spRed(0.1f));
        }
    }

    /// draw the origin and the initial 2 minkowski points
    spDebugDrawPoint(spVectorZero(), spGreen(0.2f));
    spDebugDrawPoint(m0, spBlue(0.8f));
    spDebugDrawPoint(m1, spBlue(0.8f));
#endif

    static const spInt max_iters = 16;

    for (spInt i = 0; i < max_iters; ++i)
    {
        /// check if the origin is to the right of the direction m0, m1.
        if (spGetOriginSide(m0, m1) > 0.0f)
        {
            /// its to the right, swap the minkowski points so they are on the left
            spSwap(&m0, &m1);
        }

        /// TODO: fix this for better convergence
        /// get the direction for the next support point
        spVector dir = spSkew(spSub(m0, m1));

        /// calculate the next minkowski point with the new normal direction
        spVector m2 = spSupportPoint(pa, pb, xfa, xfb, dir);

#ifdef SP_DEBUG_DRAW
        spVector p = spLerp(m0, m1, 0.5f);
        spNormalize(&dir);
        spDebugDrawLine(p, spAdd(p, dir), spPurple(0.1f));
        spDebugDrawPoint(m2, spGreen(0.8f));

        /// draw triangle
        spDebugDrawTriangle(m0, m1, m2, spYellow(0.1f));
#endif

        /// check if the origin is inside the simplex. m0, m1, m2
        if (spGetOriginSide(m0, m2) >= 0.0f && spGetOriginSide(m2, m1) >= 0.0f)
        {
            /// the origin is inside the triangle simplex, call EPA and initialize the contact
            spLog("COLLISION\n");
            return spTrue;
        }
        else
        {
            /// check if our initial edge is closer than our new minkowski point
            if (spDot(m2, dir) <= spMax(spDot(m0, dir), spDot(m1, dir)))
            {
                /// the initial edge is closer than our new minkowski point there is no collision
                spLog("initial edge is closer\n");
                break;
            }
            else
            {
                /// we need more iterations to try and get the origin inside of the simplex
                /// check which point is farther for the newest minkowski point, and remove
                /// it from the simplex
                if (spDistance(m0, m2) < spDistance(m1, m2))
                {
                    /// the first point is farther, remove it from the simple
                    m0 = m2;
                }
                else
                {
                    /// the second point is farther, remove it from the simple
                    m1 = m2;
                }
            }
        }
    }

    /// return closest points
    return spFalse;
}

spBool 
spCollidePolygons(spContact*& contact, const spCollisionInput& data)
{
    spClosestPoints points;
    if (spGJK(&points, contact, data))
    {
        //spDebugDrawPoint(points.a, spRed());
        //spDebugDrawPoint(points.b, spRed());
    }
    return spFalse;
}