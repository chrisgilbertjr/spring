
#include "spSegment.h"

void 
spSegmentInit(spSegment* segment, spVector pointA, spVector pointB, spFloat radius, spFloat mass)
{
    spShape* shape = &segment->shape;
    spMassData massData;
    spBound bound;

    /// init the segment
    segment->pointA = pointA;
    segment->pointB = pointB;
    segment->radius = radius;
    segment->tangentA = spVectorZero();
    segment->tangentB = spVectorZero();
    segment->normal = spNormal(spSkew(spSub(pointB, pointA)));

    /// init the shape base class
    spSegmentComputeMassData(segment, &massData, mass);
    spSegmentComputeBound(segment, &bound);
    spShapeInit2(shape, &massData, &bound, SP_SHAPE_SEGMENT);

    /// check if everything is init correctly
    spSegmentIsSane(segment);
}

spSegment* 
spSegmentAlloc()
{
    return (spSegment*) spMalloc(sizeof(spSegment));
}

spShape* 
spSegmentNew(spVector pointA, spVector pointB, spFloat radius, spFloat mass)
{
    spSegment* segment = spSegmentAlloc();
    spSegmentInit(segment, pointA, pointB, radius, mass);
    return (spShape*) segment;
}

void 
spSegmentFree(spSegment** segment)
{
    free(*segment);
    *segment = 0;
}

spVector 
spSegmentComputeCenterOfMass(const spSegment* segment)
{
    return spMult(spAdd(segment->pointA, segment->pointB), .5f);
}

spFloat 
spSegmentComputeInertia(const spSegment* segment, spFloat mass)
{
    /// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    spVector offset = spSegmentComputeCenterOfMass(segment);

    spFloat length = spDistance(segment->pointA, segment->pointB) + segment->radius * 2.0f;
    spFloat L = length * length + segment->radius * segment->radius;
    return (mass * L * L) / 12.0f + spLengthSquared(offset);
}

void 
spSegmentComputeBound(const spSegment* segment, spBound* bound)
{
    /// get the two segment points
    spVector pointA = segment->pointA;
    spVector pointB = segment->pointB;

    /// get the min and max vectors and calculate the delta between them
    spVector max = spVector(spMax(pointA.x, pointB.x), spMax(pointA.y, pointB.y));
    spVector min = spVector(spMin(pointA.x, pointB.x), spMin(pointA.y, pointB.y));
    spVector delta = spSub(max, min);

    /// compute the center and radius
    spVector center = spMult(spAdd(max, min), 0.5f);
    spFloat radius = delta.x > delta.y ? delta.x : delta.y;

    /// init the bound
    spBoundInit(bound, center, radius + segment->radius);
}

void 
spSegmentComputeMassData(spSegment* segment, spMassData* data, spFloat mass)
{
    spMassDataInit(
        data,
        spSegmentComputeCenterOfMass(segment),
        spSegmentComputeInertia(segment, mass),
        mass);
}

spBool 
spSegmentTestPoint(spSegment* segment, const spVector point)
{
    return spFalse;
}

#ifdef SP_DEBUG
void __spSegmentIsSane(spSegment* segment)
{
    spSane(segment->pointA);
    spSane(segment->pointB);
    spSane(segment->radius);
}
#endif