
#include "spSweepAndPrune.h"
#include "spBound.h"
#include "spShape.h"
#include "spBody.h"

spAxis g_axis = SP_X;

static int 
CompareIntervals(const void* a, const void* b)
{
    spSapBox** BoxA = (spSapBox**)a;
    spSapBox** BoxB = (spSapBox**)b;
    spSapBox* boxA = *BoxA;
    spSapBox* boxB = *BoxB;

    float minA = boxA->axis[g_axis].min;
    float minB = boxB->axis[g_axis].min;

    if (minA > minB)
    {
        return 1;
    }
    else if (minA < minB)
    {
        return -1;
    }
    return 0;
}

static spSapBox*
BoxNew(spShape* shape)
{
    spSapBox* box = (spSapBox*) spMalloc(sizeof(spSapBox));

    spInterval zero;
    zero.min = 0.0f;
    zero.max = 0.0f;

    box->shape = shape;
    box->axis[0] = zero;
    box->axis[1] = zero;

    return box;
}

static void
BoxFree(spSapBox* box)
{
    NULLCHECK(box);
    free(box);
}

static void
UpdateBox(spSapBox* box)
{
    spShape* shape = box->shape;
    spBound* bound = &shape->bound;
    spVector center = spBoundGetWorldCenter(bound, &shape->body->xf);
    spVector width = spvfMult(spBoundGetHalfWidth(bound), 3.0f);

    box->axis[SP_X].min = center.x - width.x;
    box->axis[SP_X].max = center.x + width.x;
    box->axis[SP_Y].min = center.y - width.y;
    box->axis[SP_Y].max = center.y + width.y;
}

spSap 
spSapConstruct()
{
    spSap sap;
    spMemset(sap.boxes, 0, sizeof(spSapBox**)*SP_MAX_SAP_OBJECTS);
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
    spSapBox* box = BoxNew(shape);
    sap->boxes[sap->count++] = box;
}

void 
spSapRemove(spSap* sap, spShape* shape)
{
    spInt index = -1;
    for (spInt i = 0; i < sap->count; ++i)
    {
        if (sap->boxes[i]->shape == shape)
        {
            index = i;
            break;
        }
    }

    spAssert(index >= -1, "Error: cannot remove an object that isnt in the sap list!");

    BoxFree(sap->boxes[index]);

    sap->count--;
    sap->boxes[index] = sap->boxes[sap->count];
    sap->boxes[sap->count] = NULL;
}

void 
spSapUpdate(spSap* sap)
{
    spSapUpdateSortAxis(sap);

    for (spInt i = 0; i < sap->count; ++i)
    {
        UpdateBox(sap->boxes[i]);
    }
}

void 
spSapSort(spSap* sap)
{
    qsort(sap->boxes, sap->count, sizeof(spSapBox*), CompareIntervals);
}

spVariance 
spSapVariance(spSap* sap)
{
    spVariance s, s2, variance;
    spSapBox** boxes = sap->boxes;

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
spIntervalsDontOverlap(spSapBox** boxes, spInt i, spInt j)
{
    if (boxes[j]->axis[g_axis].min > boxes[i]->axis[g_axis].max)
    {
        return spTrue;
    }
    return spFalse;
}

spBool 
spBoxesOverlap(spSapBox* a, spSapBox* b)
{
    /// check if the two intervals overlap at all
    if (a->axis[SP_X].max < b->axis[SP_X].min) return spFalse;
    if (a->axis[SP_X].min > b->axis[SP_X].max) return spFalse;
    if (a->axis[SP_Y].max < b->axis[SP_Y].min) return spFalse;
    if (a->axis[SP_Y].min > b->axis[SP_Y].max) return spFalse;

    /// they do overlap, return true
    return spTrue;
}