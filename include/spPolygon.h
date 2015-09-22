
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

/// a polygon is represented by 3 or more points, defined in CCW order.
/// a polygon can be attached to body shapes, and the vertices must 
/// form a convex shape in order to work correctly
struct spPolygon
{
    spShape shape;  ///< base shape class
    spFloat radius; ///< small radius added to polygon vertices
    spEdge* edges;  ///< array of edges. each edge contains a vertex and a normal
    spInt count;    ///< number of edges
};

/// check if a shape is a polygon
spBool spShapeIsPolygon(spShape* shape);

/// initialize a polygon
void spPolygonInit(spPolygon* poly, spVector* vertices, spInt count, spFloat mass);

/// allocate space for a polygon on the heap
spPolygon* spPolygonAlloc();

/// allocate and init a new polygon on the heap
spShape* spPolygonNew(spVector* vertices, spInt count, spFloat mass);

/// free allocated memory for a polygon from the heap
void spPolygonFree(spPolygon** poly);

/// tests if a point is inside of the polygon
spBool spPolygonTestPoint(spPolygon* poly, spVector point);

/// gets the polygons radius
spFloat spPolygonGetRadius(spPolygon* poly);

/// get the edge count of the polygon
spInt spPolygonGetCount(spPolygon* poly);

/// sets the radius of the polygon
void spPolygonSetRadius(spPolygon* poly, spFloat radius);

/// @}

#endif