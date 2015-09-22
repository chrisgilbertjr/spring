
#include "spSegment.h"

static spVector 
spSegmentComputeCenterOfMass(const spSegment* segment)
{
    NULLCHECK(segment);
    return spMult(spAdd(segment->pointA, segment->pointB), .5f);
}

static spFloat 
spSegmentComputeInertia(const spSegment* segment, spFloat mass)
{
    NULLCHECK(segment);
    /// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    spVector offset = spSegmentComputeCenterOfMass(segment);

    spFloat length = spDistance(segment->pointA, segment->pointB) + segment->radius * 2.0f;
    spFloat L = length * length + segment->radius * segment->radius;
    return (mass * L * L) / 12.0f + spLengthSquared(offset);
}

static void 
spSegmentComputeBound(const spSegment* segment, spBound* bound)
{
    NULLCHECK(segment); NULLCHECK(bound);
    /// get the two segment points
    spVector pointA = segment->pointA;
    spVector pointB = segment->pointB;

    /// get the min and max vectors and calculate the delta between them
    spVector max = spVectorConstruct(spMax(pointA.x, pointB.x), spMax(pointA.y, pointB.y));
    spVector min = spVectorConstruct(spMin(pointA.x, pointB.x), spMin(pointA.y, pointB.y));
    spVector delta = spSub(max, min);

    /// compute the center and radius
    spVector center = spMult(spAdd(max, min), 0.5f);
    spFloat radius = delta.x > delta.y ? delta.x : delta.y;

    /// init the bound
    spBoundInit(bound, center, radius + segment->radius);
}

static void 
spSegmentComputeMassData(spSegment* segment, spMassData* data, spFloat mass)
{
    NULLCHECK(segment); NULLCHECK(data);
    spMassDataInit(
        data,
        spSegmentComputeCenterOfMass(segment),
        spSegmentComputeInertia(segment, mass),
        mass);
}

spBool 
spShapeIsSegment(spShape* shape)
{
    return shape->type == SP_SHAPE_SEGMENT;
}

void 
spSegmentInit(spSegment* segment, spVector pointA, spVector pointB, spFloat radius, spFloat mass)
{
    NULLCHECK(segment);
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
    spShapeInit(shape, &massData, &bound, SP_SHAPE_SEGMENT);
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
    NULLCHECK(segment);
    spSegmentInit(segment, pointA, pointB, radius, mass);
    return (spShape*) segment;
}

void 
spSegmentFree(spSegment** segment)
{
    NULLCHECK(*segment);
    spFree(segment);
}

spBool 
spSegmentTestPoint(spSegment* segment, const spVector point)
{
    NULLCHECK(segment);
    return spFalse;
}

spVector 
spSegmentGetPointA(spSegment* segment)
{
    NULLCHECK(segment);
    return segment->pointA;
}

spVector 
spSegmentGetPointB(spSegment* segment)
{
    NULLCHECK(segment);
    return segment->pointB;
}

spVector 
spSegmentGetWorldPointA(spSegment* segment)
{
    NULLCHECK(segment);
    return spShapeLocalToWorldPoint(&segment->shape, segment->pointA);
}

spVector 
spSegmentGetWorldPointB(spSegment* segment)
{
    NULLCHECK(segment);
    return spShapeLocalToWorldPoint(&segment->shape, segment->pointB);
}

spVector 
spSegmentGetTangentA(spSegment* segment)
{
    NULLCHECK(segment);
    return segment->tangentA;
}

spVector 
spSegmentGetTangentB(spSegment* segment)
{
    NULLCHECK(segment);
    return segment->tangentB;
}

spVector 
spSegmentGetWorldTangentA(spSegment* segment)
{
    NULLCHECK(segment);
    return spShapeLocalToWorldPoint(&segment->shape, segment->tangentA);
}

spVector 
spSegmentGetWorldTangentB(spSegment* segment)
{
    NULLCHECK(segment);
    return spShapeLocalToWorldPoint(&segment->shape, segment->tangentB);
}

spVector 
spSegmentGetNormal(spSegment* segment)
{
    NULLCHECK(segment);
    return segment->normal;
}

spVector 
spSegmentGetWorldNormal(spSegment* segment)
{
    NULLCHECK(segment);
    return spShapeLocalToWorldVector(&segment->shape, segment->normal);
}

spFloat 
spSegmentGetRadius(spSegment* segment)
{
    NULLCHECK(segment);
    return segment->radius;
}

void 
spSegmentSetTangentA(spSegment* segment, spVector tangentA)
{
    NULLCHECK(segment);
    spAssert(!spEqual(tangentA, spVectorZero()), "tangentA is being set to a zero vector");
    segment->tangentA = spNormal(tangentA);
}

void 
spSegmentSetTangentB(spSegment* segment, spVector tangentB)
{
    NULLCHECK(segment);
    spAssert(!spEqual(tangentB, spVectorZero()), "tangentB is being set to a zero vector");
    segment->tangentB = spNormal(tangentB);
}

void 
spSegmentSetWorldTangentA(spSegment* segment, spVector tangentA)
{
    NULLCHECK(segment);
    spAssert(!spEqual(tangentA, spVectorZero()), "tangentA is being set to a zero vector");
    segment->tangentA = spShapeWorldToLocalPoint(&segment->shape, spNormal(tangentA));
}

void 
spSegmentSetWorldTangentB(spSegment* segment, spVector tangentB)
{
    NULLCHECK(segment);
    spAssert(!spEqual(tangentB, spVectorZero()), "tangentB is being set to a zero vector");
    segment->tangentB = spShapeWorldToLocalPoint(&segment->shape, spNormal(tangentB));
}