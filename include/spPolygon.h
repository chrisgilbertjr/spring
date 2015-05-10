
#ifndef SP_POLYGON_H
#define SP_POLYGON_H

#include "spShape.h"

/// @defgroup spPolygon spPolygon
/// @{

/// edge of a polygon, each contains a vertex and an edge normal
struct spEdge
{
    spVector vertex; ///< a polygon vertex
    spVector normal; ///< a polygon normal
};

/// used to create polygon shapes
struct spPolygonDef
{
    spMaterial material; ///< the material of the polygon
    spVector* vertices;  ///< pointer to an array of vertices. these should be allocated on the stack
    spInt vertex_count;  ///< number of vertices in the polygon
    spFloat mass;        ///< the mass of the polygon
}; 

/// TODO: document polygon
struct spPolygon
{
    spShape base_class; ///< base shape class
    spEdge* edges;      ///< array of edges. each edge contains a vertex and a normal
    spInt count;        ///< number of edges
};

/// initialize a polygon
void spPolygonInit(spPolygon* poly, spBody* body, const spPolygonDef& def);

/// allocate space for a polygon on the heap
spPolygon* spPolygonAlloc();

/// allocate and init a new polygon on the heap
spPolygon* spPolygonNew(spBody* body, const spPolygonDef& def);

/// free allocated memory for a polygon from the heap
void spPolygonFree(spPolygon*& poly);

/// compute the centroid of a polygon
spVector spPolygonComputeCenterOfMass(spPolygon* poly);

/// compute the moment of inertia for a polygon given a mass
spFloat spPolygonComputeInertia(spPolygon* poly, spFloat mass);

/// compute the bounding volume of a polygon
void spPolygonComputeBound(spPolygon* poly, spBound* bound, const spVector& com);

/// compute the mass, inertia, and center of gravity of mass for the shape
void spPolygonComputeMassData(spPolygon* poly, spMassData* data, spFloat mass);

/// sanity checks
#ifdef SP_DEBUG
 #define spEdgeIsSane(edge) _spEdgeIsSane(edge)
 #define spEdgesAreSane(edge, count) _spEdgesAreSane(edge, count)
 #define spPolygonDefIsSane(poly) _spPolygonDefIsSane(poly)
 #define spPolygonIsSane(poly) _spPolygonIsSane(poly)

 /// edge sanity check
 inline void _spEdgeIsSane(const spEdge* edge)
 {
     spAssert(spAlmostEqual(spLength(edge->normal), 1.0f, SP_EPSILON), "edge normal is not of length 1");
 }

 /// sanity check for all edges
 inline void _spEdgesAreSane(const spEdge* edges, spInt count)
 {
     spInt i = 0;
     for (const spEdge* edge = edges; i < count; edges++, i++)
     {
         _spEdgeIsSane(edge);
     }
 }

 /// polygon def sanity check
 inline void _spPolygonDefIsSane(const spPolygonDef& def)
 {
     spMaterialIsSane(def.material);
     spAssert(def.vertices != NULL, "vertices is null in polygon def sanity check");
     spAssert(def.vertex_count > 3, "a polygon must have at least 3 vertices in polygon def sanity check");
     spAssert(def.mass > 0.0f, "mass must be positive in polygon def sanity check");
 }

 /// polygon sanity check
 inline void _spPolygonIsSane(const spPolygon* poly)
 {
     spAssert(poly->count > 3, "a polygon must have at least 3 vertices");
     spEdgesAreSane(poly->edges, poly->count);
     spShapeIsSane(&poly->base_class);
 }
#else
 #define spEdgeIsSane(edge)
 #define spEdgesAreSane(edge, count)
 #define spPolygonDefIsSane(poly)
 #define spPolygonIsSane(poly)
#endif

/// @}

#endif