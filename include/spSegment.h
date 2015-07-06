
#ifndef SP_SEGMENT_H
#define SP_SEGMENT_H

#include "spShape.h"

/// @defgroup spSegment spSegment
/// @{

struct spSegment
{
    spShape shape;
    spVector pointA, pointB;
    spVector tangentA, tangentB;
    spVector normal;
    spFloat radius;
};

/// initialize a segment with two local points, a radius, and its mass
void spSegmentInit(spSegment* segment, spVector pointA, spVector pointB, spFloat radius, spFloat mass);

/// allocate a segment on the heap
spSegment* spSegmentAlloc();

/// create a new segment on the heap and init it with two local points, a radius, and its mass
spShape* spSegmentNew(spVector pointA, spVector pointB, spFloat radius, spFloat mass);

/// free a segment from the heap
void spSegmentFree(spSegment** segment);

/// check if a point is inside of the segment
spBool spSegmentTestPoint(spSegment* segment, const spVector point);

/// get the first segment point in local space
spVector spSegmentGetPointA(spSegment* segment);

/// get the second segment point in local space
spVector spSegmentGetPointB(spSegment* segment);

/// get the first segment point in world space
spVector spSegmentGetWorldPointA(spSegment* segment);

/// get the second segment point in local space
spVector spSegmentGetWorldPointB(spSegment* segment);

/// get the first tangent point in local space
spVector spSegmentGetTangentA(spSegment* segment);

/// get the second tangent point in local space
spVector spSegmentGetTangentB(spSegment* segment);

/// get the first tangent point in world space
spVector spSegmentGetWorldTangentA(spSegment* segment);

/// get the second tangent point in local space
spVector spSegmentGetWorldTangentB(spSegment* segment);

/// get the segment normal in local space
spVector spSegmentGetNormal(spSegment* segment);

/// get the segment normal in world space
spVector spSegmentGetWorldNormal(spSegment* segment);

/// get the segment radius
spFloat spSegmentGetRadius(spSegment* segment);

/// set the first segment tangent in local space
void spSegmentSetTangentA(spSegment* segment, spVector tangentA);

/// set the second segment tangent in local space
void spSegmentSetTangentB(spSegment* segment, spVector tangentA);

/// set the first segment tangent in world space
void spSegmentSetWorldTangentA(spSegment* segment, spVector tangentA);

/// set the second segment tangent in world space
void spSegmentSetWorldTangentB(spSegment* segment, spVector tangentA);

/// @}

#endif