/// license

#ifndef SP_TYPE_H
#define SP_TYPE_H

/// c headers
#include <assert.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

#define SP_DEBUG
#define SP_DEBUG_DRAW

/// @defgroup spCore spCore
/// @{

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

/// memory defines
#define spRealloc realloc
#define spMalloc malloc
#define spCalloc calloc
#define spFree(pointer) free(*pointer); *pointer = NULL;

/// boolean/ignore defines
#define SP_IGNORE(x) ((void)x)
#define spFalse 0u
#define spTrue 1u

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
typedef struct spRotation           spRotation;
typedef struct spMassData           spMassData;
typedef struct spMaterial           spMaterial;
typedef struct spSegment            spSegment;
typedef struct spPolygon            spPolygon;
typedef struct spContact            spContact;
typedef struct spVector             spVector;
typedef struct spCircle             spCircle;
typedef struct spMatrix             spMatrix;
typedef struct spFilter             spFilter;
typedef struct spShape              spShape;
typedef struct spBound              spBound;
typedef struct spWorld              spWorld;
typedef struct spEdge               spEdge;
typedef struct spBody               spBody;

/// swap a and b
inline void spSwap(spFloat* a, spFloat* b)
{
    spFloat tmp = *a;
    *a = *b;
    *b = tmp;
}

/// swap a and b
inline void spSwap(spInt* a, spInt* b)
{
    spInt tmp = *a;
    *a = *b;
    *b = tmp;
}

/// get the abs value of a
inline spFloat spAbs(const spFloat a)
{
    return a > 0.0f ? a : -a;
}

/// get the min of a and b
inline spFloat spMin(const spFloat a, const spFloat b)
{
    return a < b ? a : b;
}

/// get the max of a and b
inline spFloat spMax(const spFloat a, const spFloat b)
{
    return a > b ? a : b;
}

/// clamp x between min and max
inline spFloat spClamp(const spFloat x, const spFloat min, const spFloat max)
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
inline spBool spAlmostEqual(const spFloat a, const spFloat b, const spFloat EPSILON = 1e-6)
{
     return (b - EPSILON) <= a && a <= (b + EPSILON) ? spTrue : spFalse;
}

/// i could make this faster copying more than 1 byte at a time, but i dont think its necessary
inline void* spMemset(void* mem, spInt value, spSize bytes)
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

#ifdef SP_DEBUG
    #define NULLCHECK(ptr) spAssert(ptr != NULL, "pointer is NULL!\n")
    #define NANCHECK(val)  spAssert(val == val,  "value is NaN!\n")
    //#define spAssert  DEBUGAssert
    #define SP_LOG(file, msg)        \
        va_list args;                \
        va_start(args, msg);         \
        DEBUGDoLog(file, msg, args); \
        va_end(args);

    #define spAssert(condition, msg)  \
        if (condition == spFalse)\
        {\
            SP_LOG(stderr, msg);\
            assert(false);\
        }

    #define spWarning DEBUGWarning
    #define spLog     DEBUGLog

    /// TODO:
	void DEBUGDoLog(FILE* file, const char* msg, va_list args);

	/// TODO:
	void DEBUGAssert(spBool condition, const char* msg, ...);

	/// TODO:
	void DEBUGWarning(spBool condition, const char* msg, ...);

	/// TODO:
	void DEBUGLog(const char* msg, ...);
#else
    #define NULLCHECK(ptr)
    #define NANCHECK(val)
    #define spAssert  
    #define spWarning 
    #define spLog     
#endif

/// @}

#endif