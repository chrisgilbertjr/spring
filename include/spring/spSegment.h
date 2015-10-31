
#ifndef SP_SEGMENT_H
#define SP_SEGMENT_H

#include "spShape.h"

/// @defgroup spSegment spSegment
/// @{

/// a segment is represented by two points. they create an edge with a normal and optional tangents for collision rejection
struct spSegment
{
    spShape shape;               ///< base shape class
    spVector pointA, pointB;     ///< two local space points for body A and body B
    spVector tangentA, tangentB; ///< two local space tangents for body A and body B
    spVector normal;             ///< segment edge normal
    spFloat radius;              ///< small segment radius
};

/// check if a shape is a segment
SPRING_API spBool spShapeIsSegment(spShape* shape);

/// initialize a segment with two local points, a radius, and its mass
SPRING_API void spSegmentInit(spSegment* segment, spVector pointA, spVector pointB, spFloat radius, spFloat mass);

/// allocate a segment on the heap
SPRING_API spSegment* spSegmentAlloc();

/// create a new segment on the heap and init it with two local points, a radius, and its mass
SPRING_API spShape* spSegmentNew(spVector pointA, spVector pointB, spFloat radius, spFloat mass);

/// free a segment from the heap
SPRING_API void spSegmentFree(spShape** segment);

/// check if a point is inside of the segment
SPRING_API spBool spSegmentTestPoint(spSegment* segment, const spVector point);

/// get the first segment point in local space
SPRING_API spVector spSegmentGetPointA(spSegment* segment);

/// get the second segment point in local space
SPRING_API spVector spSegmentGetPointB(spSegment* segment);

/// get the first segment point in world space
SPRING_API spVector spSegmentGetWorldPointA(spSegment* segment);

/// get the second segment point in local space
SPRING_API spVector spSegmentGetWorldPointB(spSegment* segment);

/// get the first tangent point in local space
SPRING_API spVector spSegmentGetTangentA(spSegment* segment);

/// get the second tangent point in local space
SPRING_API spVector spSegmentGetTangentB(spSegment* segment);

/// get the first tangent point in world space
SPRING_API spVector spSegmentGetWorldTangentA(spSegment* segment);

/// get the second tangent point in local space
SPRING_API spVector spSegmentGetWorldTangentB(spSegment* segment);

/// get the segment normal in local space
SPRING_API spVector spSegmentGetNormal(spSegment* segment);

/// get the segment normal in world space
SPRING_API spVector spSegmentGetWorldNormal(spSegment* segment);

/// get the segment radius
SPRING_API spFloat spSegmentGetRadius(spSegment* segment);

/// set the first segment tangent in local space
SPRING_API void spSegmentSetTangentA(spSegment* segment, spVector tangentA);

/// set the second segment tangent in local space
SPRING_API void spSegmentSetTangentB(spSegment* segment, spVector tangentA);

/// set the first segment tangent in world space
SPRING_API void spSegmentSetWorldTangentA(spSegment* segment, spVector tangentA);

/// set the second segment tangent in world space
SPRING_API void spSegmentSetWorldTangentB(spSegment* segment, spVector tangentA);

/// @}

#endif