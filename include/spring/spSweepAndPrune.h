
#ifndef SP_SORT_AND_SWEEP_H
#define SP_SORT_AND_SWEEP_H

#include "spBound.h"

#define SP_MAX_SAP_OBJECTS 512

/// interval along a 1D axis
struct spInterval
{
    spFloat min, max; /// min and max values along an axis
};

/// sap box entry in the broadphase, min bound sorted along the x or y axis
struct spSapBox
{
    spShape* shape;
    spInterval axis[2];
};

/// sap broadphase
struct spSap
{
    spSapBox* boxes[SP_MAX_SAP_OBJECTS]; /// array of boxes @TODO: change to dynamic array in the future
    spInt count; /// count of objects in the array
};

typedef struct 
{
    spFloat axis[2];
} spVariance;

SPRING_API extern spAxis g_axis; /// axis to sort against (highest variance)

/// construct a new sweep and prune broadphase
SPRING_API spSap spSapConstruct();

/// destroy a sweep and prune broadphase and release all resources
SPRING_API spSap spSapDestroy(spSap* sap);

/// insert a shape in the sweep and prune broadphase
SPRING_API void spSapInsert(spSap* sap, spShape* shape);

/// remove a shape in the sweep and prune broadphase
SPRING_API void spSapRemove(spSap* sap, spShape* shape);

/// update each box in the broadphase by updating the interval to world space
SPRING_API void spSapUpdate(spSap* sap);

/// sort each box in the broadphase using quicksort for now (radix sort can improve speed in some situations)
SPRING_API void spSapSort(spSap* sap);

/// compute the variance, and set new axis accordingly to help reduce o(n^2) during clustering
SPRING_API spVariance spSapVariance(spSap* sap);

/// update to have SAP sort on the axis with the highest variance
SPRING_API void spSapUpdateSortAxis(spSap* sap);

/// check if two intervals do not overlap
SPRING_API spBool spIntervalsDontOverlap(spSapBox** boxes, spInt i, spInt j);

/// check if two boxes overlap
SPRING_API spBool spBoxesOverlap(spSapBox* a, spSapBox* b);

#endif