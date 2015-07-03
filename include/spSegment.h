
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

/// TODO:
void spSegmentInit(spSegment* segment, spVector pointA, spVector pointB, spFloat radius, spFloat mass);

/// TODO:
spSegment* spSegmentAlloc();

/// TODO:
spShape* spSegmentNew(spVector pointA, spVector pointB, spFloat radius, spFloat mass);

/// TODO:
void spSegmentFree(spSegment** segment);

/// TODO:
spVector spSegmentComputeCenterOfMass(const spSegment* segment);

/// TODO:
spFloat spSegmentComputeInertia(const spSegment* segment, spFloat mass);

/// TODO:
void spSegmentComputeBound(const spSegment* segment, spBound* bound);

/// TODO:
void spSegmentComputeMassData(spSegment* segment, spMassData* data, spFloat mass);

/// TODO:
spBool spSegmentTestPoint(spSegment* segment, const spVector point);

/// TODO: sanity checks
#ifdef SP_DEBUG
    #define spSegmentIsSane(segment) __spSegmentIsSane(segment);
    void __spSegmentIsSane(spSegment* segment);
#else
    #define spSegmentIsSane(segment)
#endif

/// @}

#endif