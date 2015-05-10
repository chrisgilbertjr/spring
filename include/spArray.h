
#ifndef SP_ARRAY_H
#define SP_ARRAY_H

#include "spCore.h"

/// TODO:
struct spArray
{
    spInt size;
    spInt index;
    spLazyPointer* data;
};

#endif