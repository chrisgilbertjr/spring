
#include "spCore.h"

void spDoLog(FILE* file, const char* msg, va_list args)
{
    vfprintf(file, msg, args);
}

void _spAssert(spBool condition, const char* msg, ...)
{
    if (condition == spFalse)
    {
        SP_LOG(stderr, msg);
        assert(false);
    }
}

void spSane(spFloat val)
{
    spAssert(val == val, "NaN!\n");
}