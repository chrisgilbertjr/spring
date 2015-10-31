
#include "spCore.h"

/// debug functions
#ifndef NDEBUG

    void DEBUGDoLog(FILE* file, const char* msg, va_list args)
    {
        vfprintf(file, msg, args);
    }

    void DEBUGAssert(spBool condition, const char* msg, ...)
    {
        if (condition == spFalse)
        {
            SP_LOG(stderr, msg);
            assert(spFalse);
        }
    }

    void DEBUGWarning(spBool condition, const char* msg, ...)
    {
        SP_LOG(stderr, msg);
    }

    void DEBUGLog(const char* msg, ...)
    {
        SP_LOG(stdout, msg);
    }
#endif