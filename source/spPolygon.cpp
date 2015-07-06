
#include "spPolygon.h"
#include "spBody.h"

static spVector 
spPolygonComputeCenterOfMass(spPolygon* poly)
{
    NULLCHECK(poly);
    /// http://en.wikipedia.org/wiki/Centroid#Centroid_of_polygon 
    ///
    /// compute the centroid
    /// E = sum of each expression from 1 to n
    /// P = vertices in ccw order
    /// A = (Pn + Pn+1)
    /// B = cross(Pn, Pn+1)
    /// N = E(A * B)
    /// D = E(B)
    /// centroid of poly = N / (3.0 * D)

    spVector N = spVectorZero();
    spFloat  D = 0.0f;
    spInt count = poly->count;
    for (spInt i = 0; i < count; ++i)
    {
        spVector v0 = poly->edges[i].vertex;
        spVector v1 = poly->edges[(i+1) % count].vertex;

        spVector A = spAdd(v0, v1);
        spFloat  B = spCross(v0, v1);

        N  = spAdd(N, spMult(A, B));
        D += B;
    }
    return spMult(N, 1.0f/(D*3.0f));
}

static spFloat
spPolygonComputeInertia(spPolygon* poly, spFloat mass)
{
    NULLCHECK(poly);
    spAssert(poly != NULL, "the polygon is null while computing the moment of inertia");

    /// http://en.wikipedia.org/wiki/List_of_moments_of_inertia
    ///
    /// compute moment of inertia
    /// E = sum of each expression from 1 to n
    /// P = vertices in ccw order
    /// A = cross(Pn+1, Pn)
    /// B = dot(Pn+1, Pn+1) + dot(Pn+1, Pn) + dot(Pn, Pn)
    /// M = mass of poly
    /// N = E(A * B)
    /// D = E(B)
    /// inertia for convex plane poly = (M * N) / (6.0 * D)

    spFloat N = 0.0f;
    spFloat D = 0.0f;
    spInt count = poly->count;
    for (spInt i = 0; i < count; ++i)
    {
        spVector v0 = poly->edges[i].vertex;
        spVector v1 = poly->edges[(i+1) % count].vertex;

        spFloat A = spCross(v0, v1);
        spFloat B = spDot(v1, v1) + spDot(v1, v0) + spDot(v0, v0);

        N += A * B;
        D += B;
    }

    return ((mass * N) / (6.0f * D)) * SP_DEG_TO_RAD;
}

static void 
spPolygonComputeBound(spPolygon* poly, spBound* bound, const spVector center)
{
    NULLCHECK(poly); NULLCHECK(bound);
    /// TODO: im computing a fat bounding box in this test, 
    /// should i compute a tight fight a recompute often?
    spInt count = poly->count;
    spFloat max = SP_MIN_FLT;
    for (spInt i = 0; i < count; ++i)
    {
        spVector v0 = poly->edges[i].vertex;

        /// calculate the distance between each vertex and the center of the poly
        spFloat length = spDistance(v0, center);

        /// get the max distance from the center to a each vertex
        if (length > max)
        {
            max = length;
        }
    }

    /// intialize the bound with a center and radius (fat AABB)
    spBoundInit(bound, center, max);
}

static void 
spPolygonComputeMassData(spPolygon* poly, spMassData* data, spFloat mass)
{
    NULLCHECK(poly); NULLCHECK(data);
    spAssert(mass >= 0.0f, "mass is negative while computing mass data");

    /// initialize all mass related data
    spMassDataInit(
        data, 
        spPolygonComputeCenterOfMass(poly), 
        spPolygonComputeInertia(poly, mass), 
        mass);
}

void 
spPolygonInit(spPolygon* poly, spVector* vertices, spInt count, spFloat mass)
{
    NULLCHECK(poly); NULLCHECK(vertices);
    spAssert(count > 2, "less than 3 vertices in spPolygonInit\n");
    spMaterial material = { 0.6f, 0.2f };

    poly->count = count;
    poly->edges = (spEdge*) spMalloc(sizeof(spEdge) * count);
    poly->radius = 0.55f;
    NULLCHECK(poly->edges);

    /// initialize vertices and normals
    for (spInt i = 0; i < count; ++i)
    {
        spVector tail = vertices[i]; /// get this edge vertex
        spVector head = vertices[(i+1) % count]; /// get the next, with a wrapping index
        spVector normal = spNormal(spSkewT(spSub(head, tail)));

        poly->edges[i].vertex = tail;
        poly->edges[i].normal = normal;
    }

    spMassData mass_data;
    spBound    bound;

    spPolygonComputeMassData(poly, &mass_data, mass);
    spPolygonComputeBound(poly, &bound, mass_data.com);

    spShapeInit(&poly->shape, &mass_data, &bound, SP_SHAPE_POLYGON);
}

spShape* 
spPolygonNew(spVector* vertices, spInt count, spFloat mass)
{
    NULLCHECK(vertices);
    spPolygon* poly = spPolygonAlloc();
    spPolygonInit(poly, vertices, count, mass);
    return (spShape*)poly;
}

spPolygon* 
spPolygonAlloc()
{
    return (spPolygon*) spMalloc(sizeof(spPolygon));
}

void 
spPolygonFree(spPolygon** poly)
{
    NULLCHECK(poly);
    spPolygon* polygon = *poly;
    NULLCHECK(polygon);
    spEdge** edges = &polygon->edges;
    NULLCHECK(edges);
    spFree(edges);
    spFree(poly);
}

spBool 
spPolygonTestPoint(spPolygon* poly, spVector point)
{
    NULLCHECK(poly);
    spTransform* xf = &poly->shape.body->xf;
    spVector v0 = spTMult(*xf, point);

    spInt count = poly->count;
    for (spInt i = 0; i < count; ++i)
    {
        spVector v1 = poly->edges[i].vertex;
        spVector v2 = poly->edges[(i+1) % count].vertex;

        spVector A = spSub(v1, v0);
        spVector B = spSub(v2, v0);

        spMatrix mat = spMatrix(A.x, B.x, A.y, B.y);
        if (spDeterminant(mat) < 0.0f) return spFalse;
    }
    return spTrue;
}

spFloat 
spPolygonGetRadius(spPolygon* poly)
{
    NULLCHECK(poly);
    return poly->radius;
}

spInt 
spPolygonGetCount(spPolygon* poly)
{
    NULLCHECK(poly);
    return poly->count;
}

void 
spPolygonSetRadius(spPolygon* poly, spFloat radius)
{
    NULLCHECK(poly);
    poly->radius = radius;
}
