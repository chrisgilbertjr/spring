
#include "spPolygon.h"

void 
spPolygonInit(spPolygon* poly, spBody* body, const spPolygonDef& def)
{
    spPolygonDefIsSane(def);

    const spMaterial* material = &def.material;
    spVector* vertices = def.vertices;
    spInt count = def.vertex_count;
    spFloat mass = def.mass;

    poly->count = count;
    poly->edges = (spEdge*) spMalloc(sizeof(spEdge) * count);

    /// initialize vertices and normals
    for (spInt i = 0; i < count; ++i)
    {
        spVector tail = vertices[i]; /// get this edge vertex
        spVector head = vertices[(i+1) % count]; /// get the next, with a wrapping index
        spVector normal = spSkewT(spSub(head, tail));
        spNormalize(&normal);

        poly->edges[i].vertex = tail;
        poly->edges[i].normal = normal;
    }

    spShapeDef shape_def;
    spMassData mass_data;
    spBound    bound;

    spPolygonComputeMassData(poly, &mass_data, mass);
    spPolygonComputeBound(poly, &bound, mass_data.com);

    shape_def.body = body;
    shape_def.type = SP_SHAPE_POLYGON;
    shape_def.mass_data = &mass_data;
    shape_def.material = material;
    shape_def.bound = &bound;

    spShapeInit(&poly->base_class, shape_def);

    spPolygonIsSane(poly);
}

spPolygon* 
spPolygonNew(spBody* body, const spPolygonDef& def)
{
    spPolygon* poly = spPolygonAlloc();
    spPolygonInit(poly, body, def);
    return poly;
}

spPolygon* 
spPolygonAlloc()
{
    return (spPolygon*) spMalloc(sizeof(spPolygon));
}

void 
spPolygonFree(spPolygon*& poly)
{
    spAssert(poly->edges != NULL, "the polygon edges are NULL while freeing memory!");
    spFree(poly->edges);
    spFree(poly);
}

spVector 
spPolygonComputeCenterOfMass(spPolygon* poly)
{
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

spFloat
spPolygonComputeInertia(spPolygon* poly, spFloat mass)
{
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
    return  (mass * N) / (6.0f * D);
}

void 
spPolygonComputeBound(spPolygon* poly, spBound* bound, const spVector& center)
{
    spAssert(poly  != NULL, "the polygon is null while computing a bounding volume");
    spAssert(bound != NULL, "bound is null while computing a polygon bound");

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

void 
spPolygonComputeMassData(spPolygon* poly, spMassData* data, spFloat mass)
{
    spAssert(poly != NULL, "the polygon is null while computing mass data");
    spAssert(data != NULL, "mass data is null while computing mass data");
    spAssert(mass >= 0.0f, "mass is negative while computing mass data");

    /// initialize all mass related data
    spMassDataInit(
        data, 
        spPolygonComputeCenterOfMass(poly), 
        spPolygonComputeInertia(poly, mass), 
        mass);
}
