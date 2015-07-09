/// license

#ifndef SP_TYPE_H
#define SP_TYPE_H

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

#define SP_EPSILON 1e-6f
#define SP_FLT_EPSILON FLT_EPSILON
#define SP_PI 3.1415926535897932f
#define SP_RAD_TO_DEG 180.f / 3.1415926535897932f
#define SP_DEG_TO_RAD 3.1415926535897932f / 180.f

#ifdef INFINITY
    #define SP_INFINITY INFINITY
#else
    #define SP_INFINITY 1e1000
#endif



#define SP_IGNORE(x) ((void)x)
#define spRealloc realloc
#define spMalloc malloc
#define spCalloc calloc
#define spFree(pointer) free(*pointer); *pointer = NULL;
#define spFalse 0u
#define spTrue 1u

/// faked constructors for easy stack allocation
#define spVector(x, y)                  _spVector(x, y)
#define spVectorZero()                  _spVector(0.0f, 0.0f)
#define spMatrix(a, b ,c, d)            _spMatrix(a, b, c, d)
#define spMatrixZero()                  _spMatrix(0.0f, 0.0f, 0.0f, 0.0f)
#define spMatrixIdentity()              _spMatrix(1.0f, 0.0f, 0.0f, 1.0f)
#define spMatrixDiagonal(a)             _spMatrix(a, 0.0f, 0.0f, a);
#define spRotation(a)                   _spRotation(a)
#define spRotationZero()                _spRotation(0.0f)
#define spRotationRadians(s, c)         _spRotation(s, c)
#define spTransform(p, q)               _spTransform(p, q)
#define spCollisionMatrix()             _spCollisionMatrix()
#define spCollisionInput(a, b, xa, xb)  _spCollisionInput(a, b, xa, xb)
#define spWorld(g)                      _spWorld(g)
#define spContactKey(a, b)              _spContactKey(a, b)
#define spBodyDef()                     _spBodyDef()

/// for each iters
#define for_each_constraint(joint, initializer) for (spConstraint* joint = initializer; joint != NULL; joint = joint->next)
#define for_each_contact(contact, initializer) for (spContact* contact = initializer; contact != NULL; contact = contact->next)
#define for_each_shape(shape, initializer) for (spShape* shape = initializer; shape; shape = shape->next)
#define for_each_body(body, initializer) for (spBody* body = initializer; body; body = body->next)

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
typedef struct spContactPoint spContactPoint;
typedef struct spBroadPhase spBroadPhase;
typedef struct spConstraint spConstraint;
typedef struct spTransform spTransform;
typedef struct spRotation spRotation;
typedef struct spContact spContact;
typedef struct spCluster spCluster;
typedef struct spVector spVector;
typedef struct spMatrix spMatrix;
typedef struct spWorld spWorld;
typedef struct spShape spShape;
typedef struct spBody spBody;
typedef struct spBound spBound;
typedef struct spMassData spMassData;

inline void spSwap(spFloat* a, spFloat* b)
{
    spFloat tmp = *a;
    *a = *b;
    *b = tmp;
}

inline void spSwap(spInt* a, spInt* b)
{
    spInt tmp = *a;
    *a = *b;
    *b = tmp;
}

inline spFloat spAbs(const spFloat a)
{
    return a > 0.0f ? a : -a;
}

inline spFloat spMin(const spFloat a, const spFloat b)
{
    return a < b ? a : b;
}

inline spFloat spMax(const spFloat a, const spFloat b)
{
    return a > b ? a : b;
}

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

inline spBool spAlmostEqual(const spFloat a, const spFloat b, const spFloat EPSILON = SP_FLT_EPSILON)
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

inline spFloat spDeg2Rad(spFloat val)
{
    return val * SP_DEG_TO_RAD;
}

inline spFloat spRad2Deg(spFloat val)
{
    return val * SP_DEG_TO_RAD;
}

/// @}

/// @defgroup spDebug spDebug
/// @{

#ifdef SP_DEBUG
    #define NULLCHECK(ptr) spAssert(ptr != NULL, "pointer is NULL!\n")
    #define NANCHECK(val)  spAssert(val == val,  "value is NaN!\n")
    #define spAssert  DEBUGAssert
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