
#include "spCore.h"

/// debug functions
#ifdef SP_DEBUG
//    #define SP_LOG(file, msg)        \
//        va_list args;                \
//        va_start(args, msg);         \
//        DEBUGDoLog(file, msg, args); \
//        va_end(args);

    void DEBUGDoLog(FILE* file, const char* msg, va_list args)
    {
        vfprintf(file, msg, args);
    }

    void DEBUGAssert(spBool condition, const char* msg, ...)
    {
        if (condition == spFalse)
        {
            SP_LOG(stderr, msg);
            assert(false);
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