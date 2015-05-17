
#ifndef SP_CHAIN_H
#define SP_CHAIN_H

#include "spShape.h"

/// @defgroup spChain spChain
/// @{

struct spChain
{
    spShape base_class;
};

/// initialize a chain struct
void spChainInit(spChain* chain);

/// 'faked' constructor for stack allocation
spChain _spChain();

/// allocate space for a new chain on the heap
spChain* spChainAlloc();

/// allocate and initialize a new chain on the heap
spChain* spChainNew();

/// @}

#endif