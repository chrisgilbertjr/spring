/// license

#ifndef SP_CORE_H
#define SP_CORE_H

/// c headers
#include <assert.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

/// @defgroup spCore spCore
/// @{

#include "spPlatform.h"

/// memory defines
#define spRealloc realloc
#define spMalloc malloc
#define spCalloc calloc
#define spFree(pointer) free(*pointer); *pointer = NULL;

/// boolean/ignore defines
#define SP_IGNORE(x) ((void)x)
#define spFalse 0u
#define spTrue 1u

/// precision preference/defines
#ifdef DOUBLE_PRECISION
  typedef double spFloat;
  #define spatan2 atan2
  #define spfloor floor
  #define spacos acos
  #define spceil ceil
  #define spsqrt sqrt
  #define spcos cos
  #define spexp exp
  #define spsin sin
  #define sppow pow
  #define SP_MAX_FLT DBL_MAX
  #define SP_MIN_FLT DBL_MIN
#else
  typedef float spFloat;
  #define spatan2 atan2f
  #define spfloor floorf
  #define spacos acosf
  #define spceil ceilf
  #define spsqrt sqrtf
  #define spcos cosf
  #define spexp expf
  #define spsin sinf
  #define sppow powf
  #define SP_MAX_FLT FLT_MAX
  #define SP_MIN_FLT FLT_MIN
#endif

/// spring data types
typedef char           spInt8;
typedef short          spInt16;
typedef int            spInt;
typedef int            spSize;
typedef unsigned char  spUint8;
typedef unsigned short spUint16;
typedef unsigned int   spUint;
typedef unsigned int   spMask;
typedef unsigned int   spBool;
typedef unsigned int   spGroup;
typedef void*          spLazyPointer;

/// struct typedefs for convenience
typedef struct spAngularSpringJoint spAngularSpringJoint;
typedef struct spCollisionResult    spCollisionResult;
typedef struct spConstraintFuncs    spConstraintFuncs;
typedef struct spDistanceJoint      spDistanceJoint;
typedef struct spContactPoint       spContactPoint;
typedef struct spSpringJoint        spSpringJoint;
typedef struct spWheelJoint         spWheelJoint;
typedef struct spContactKey         spContactKey;
typedef struct spMotorJoint         spMotorJoint;
typedef struct spMouseJoint         spMouseJoint;
typedef struct spPointJoint         spPointJoint;
typedef struct spConstraint         spConstraint;
typedef struct spTransform          spTransform;
typedef struct spRopeJoint          spRopeJoint;
typedef struct spGearJoint          spGearJoint;
typedef struct spInterval           spInterval;
typedef struct spRotation           spRotation;
typedef struct spMassData           spMassData;
typedef struct spMaterial           spMaterial;
typedef struct spSegment            spSegment;
typedef struct spPolygon            spPolygon;
typedef struct spContact            spContact;
typedef struct spVector             spVector;
typedef struct spCircle             spCircle;
typedef struct spMatrix             spMatrix;
typedef struct spSapBox             spSapBox;
typedef struct spFilter             spFilter;
typedef struct spShape              spShape;
typedef struct spBound              spBound;
typedef struct spWorld              spWorld;
typedef struct spEdge               spEdge;
typedef struct spBody               spBody;
typedef struct spSap                spSap;

/// swap a and b
INLINE void spSwapFloat(spFloat* a, spFloat* b)
{
    spFloat tmp = *a;
    *a = *b;
    *b = tmp;
}

/// swap a and b
INLINE void spSwapInt(spInt* a, spInt* b)
{
    spInt tmp = *a;
    *a = *b;
    *b = tmp;
}

/// get the abs value of a
INLINE spFloat spAbs(const spFloat a)
{
    return a > 0.0f ? a : -a;
}

/// get the min of a and b
INLINE spFloat spMin(const spFloat a, const spFloat b)
{
    return a < b ? a : b;
}

/// get the max of a and b
INLINE spFloat spMax(const spFloat a, const spFloat b)
{
    return a > b ? a : b;
}

/// clamp x between min and max
INLINE spFloat spClamp(const spFloat x, const spFloat min, const spFloat max)
{
    if (x < min)
    {
        return min;
    }
    if (x > max)
    {
        return max;
    }
    return x;
}

/// check if two floats are equal within an epsilon value
INLINE spBool spAlmostEqualFlts(const spFloat a, const spFloat b)
{
     return (b - 1e-6) <= a && a <= (b + 1e-6) ? spTrue : spFalse;
}

/// i could make this faster copying more than 1 byte at a time, but i dont think its necessary
INLINE void* spMemset(void* mem, spInt value, spSize bytes)
{
    spUint8* ptr = (spUint8*)mem;

    while(bytes--)
    {
        *ptr++ = (spUint8)value;
    }
    return mem;
}

/// @}

/// @defgroup spDebug spDebug
/// @{

#ifdef NDEBUG
    #define NULLCHECK(ptr)
    #define NANCHECK(val)
    #define spAssert  
    #define spWarning 
    #define spLog  
#else
    #define NULLCHECK(ptr) spAssert(ptr != NULL, "pointer is NULL!\n")
    #define NANCHECK(val)  spAssert(val == val,  "value is NaN!\n")
    //#define spAssert  DEBUGAssert
    #define SP_LOG(file, msg)        \
        va_list args;                \
        va_start(args, msg);         \
        DEBUGDoLog(file, msg, args); \
        va_end(args);

    #define spAssert  DEBUGAssert
    #define spWarning DEBUGWarning
    #define spLog     DEBUGLog

    /// TODO:
	SPRING_API void DEBUGDoLog(FILE* file, const char* msg, va_list args);

	/// TODO:
	SPRING_API void DEBUGAssert(spBool condition, const char* msg, ...);

	/// TODO:
	SPRING_API void DEBUGWarning(spBool condition, const char* msg, ...);

	/// TODO:
	SPRING_API void DEBUGLog(const char* msg, ...);
#endif

/// @}

#endif