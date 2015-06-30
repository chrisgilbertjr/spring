
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
    spFloat radius;
};

/// TODO:
void spSegmentInit(spSegment* segment);

/// TODO:
spSegment* spSegmentAlloc();

/// TODO:
spSegment* spSegmentNew();

/// TODO:
void spSegmentFree(spSegment** segment);

/// TODO:
spVector spSegmentComputeCenterOfMass(const spSegment* segment);

/// TODO:
spFloat spSegmentComputeInertia(const spSegment* segment);

/// TODO:
void spSegmentComputeBound(const spSegment* segment, spBound* bound, const spVector com);

/// TODO:
void spSegmentComputeMassData(spSegment* segment, spMassData* data, spFloat mass);

/// TODO:
spBool spSegmentTestPoint(spSegment* segment, const spVector point);

/// TODO: sanity checks

/// @}

#endif