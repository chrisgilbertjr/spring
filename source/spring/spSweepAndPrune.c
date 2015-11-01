
#include "spSweepAndPrune.h"

spAxis g_axis = SP_X;

static int 
CompareIntervals(const void* a, const void* b)
{
    spBox* boxA = (spBox*)a;
    spBox* boxB = (spBox*)b;

    float minA = boxA->axis[g_axis].min;
    float minB = boxB->axis[g_axis].min;

    if (minA > minB)
    {
        return 1;
    }
    return -1;
}

static spBox*
BoxNew(spShape* shape)
{
    spBox* box = (spBox*) spMalloc(sizeof(spBox));

    spInterval zero;
    zero.min = 0.0f;
    zero.max = 0.0f;

    box->shape = shape;
    box->axis[0] = zero;
    box->axis[1] = zero;

    return box;
}

static spBox*
BoxFree(spBox** box)
{
    NULLCHECK(box);
    spFree(box);
}

static void
UpdateBox(spBox* box)
{
}

spSap 
spSapConstruct()
{
    spSap sap;
    spMemset(sap.boxes, 0, sizeof(spBox*)*SP_MAX_SAP_OBJECTS);
    sap.count = 0;

    return sap;
}

spSap 
spSapDestroy(spSap* sap)
{
    for (spInt i = sap->count; i >= 0; ++i)
    {
        BoxFree(sap->boxes[i]);
    }
    sap->count = 0;
}

void 
spSapInsert(spSap* sap, spShape* shape)
{
    /// @TODO:
}

void 
spSapRemove(spSap* sap, spShape* shape)
{
    /// @TODO:
}

void 
spSapUpdate(spSap* sap)
{
    for (spInt i = 0; i < sap->count; ++i)
    {
        UpdateBox(sap->boxes[i]);
    }
}

void 
spSapSort(spSap* sap)
{
    qsort(sap->boxes, sap->count, sizeof(spBox*), CompareIntervals);
}

spVariance 
spSapVariance(spSap* sap)
{
    spVariance s, s2, variance;
    spBox** boxes = sap->boxes;

    for (spInt i = 0; i < sap->count; ++i)
    {
        for (spInt j = 0; j < 2; ++j)
        {
            spInterval* interval = &boxes[i]->axis[j];
            spFloat center = 0.5f * (interval->min + interval->max);

            s.axis[j]  += center;
            s2.axis[j] += center * center;
        }
    }

    spFloat invObjects = 1 / (spFloat)sap->count;
    for (spInt i = 0; i < 2; ++i)
    {
        variance.axis[i] = s2.axis[i] - s.axis[i] * s.axis[i] * invObjects;
    }

    return variance;
}

void 
spSapUpdateSortAxis(spSap* sap)
{
    spVariance variance = spSapVariance(sap);

    g_axis = SP_X;
    if (variance.axis[SP_Y] > variance.axis[SP_X])
    {
        g_axis = SP_Y;
    }
}

spBool 
spIntervalsDontOverlap(spBox** boxes, spInt i, spInt j)
{
    if (boxes[j]->axis[g_axis].min > boxes[i]->axis[g_axis].max)
    {
        return spTrue;
    }
    return spFalse;
}