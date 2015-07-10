/// license

#ifndef SP_MATH_H
#define SP_MATH_H

#include "spCore.h"

/// math constants
#define SP_EPSILON 1e-6f
#define SP_FLT_EPSILON FLT_EPSILON
#define SP_PI 3.1415926535897932f
#define SP_RAD_TO_DEG 180.f / 3.1415926535897932f
#define SP_DEG_TO_RAD 3.1415926535897932f / 180.f
#define SP_INFINITY 1e10f

/// math defines
#define spVectorZero()  spVectorConstruct(0.0f, 0.0f)
#define spMatrixZero()  spMatrixConstruct(0.0f, 0.0f, 0.0f, 0.0f)
#define spRotationZero() spRotationConstruct(0.0f)

/// easy access to matrix axis
enum spAxis
{
    x = 0,
    y = 1,
};

/// @defgroup spVector spVector
/// @{

/// 2 component vector
struct spVector
{
    spFloat x, y;
};

/// @}

/// @defgroup spMatrix spMatrix
/// @{

/// 2x2 matrix 
struct spMatrix
{
    spFloat a, b, c, d;
};

/// @}

/// @defgroup spRotation spRotation
/// @{

struct spRotation
{
    spFloat s; ///< sin of the rotation in radians
    spFloat c; ///< cos of the rotation in radians
};

/// @}

/// @defgroup spTransform spTransform
/// @{

struct spTransform
{
    spVector   p; ///< position
    spRotation q; ///< rotation
};

/// @}

/// @ingroup spVector
/// @{

/// swap two vectors
inline void spSwap(spVector* a, spVector* b)
{
    spVector tmp = *a;
    *a = *b;
    *b = *b;
}

/// stack constructor
inline spVector spVectorConstruct(const spFloat x, const spFloat y)
{
    return 
    { 
        x, /// x
        y  /// y
    };
}

/// vector setter
inline void spVectorSet(spVector* a, const spFloat x, const spFloat y)
{
    a->x = x;
    a->y = y;
}

/// check for equality between two vectors
inline spBool spEqual(const spVector a, const spVector b)
{
    return a.x == b.x && a.y == b.y;
}

/// checks if a is less than b component wise
inline spBool spLessThan(const spVector a, const spVector b)
{
    return a.x < b.x && a.y < b.y;
}

/// dot product between vector a and b
inline spFloat spDot(const spVector a, const spVector b)
{
    return a.x*b.x + a.y*b.y;
}

/// returns the length of the vector
inline spFloat spLength(const spVector a)
{
    return spsqrt(spDot(a, a));
}

/// returns the squared length of the vector
inline spFloat spLengthSquared(const spVector a)
{
    return spDot(a, a);
}

/// skew a vector to be perp with itself (counter-clockwise rotation)
inline spVector spSkew(const spVector a)
{
    return spVectorConstruct(-a.y, a.x);
}

/// transpose skew a vector to be perp with itself (clockwise rotation)
inline spVector spSkewT(const spVector a)
{
    return spVectorConstruct(a.y, -a.x);
}

/// add two vectors component wise
inline spVector spAdd(const spVector a, const spVector b)
{
    return spVectorConstruct(a.x+b.x, a.y+b.y);
}

/// subtract two vectors component wise
inline spVector spSub(const spVector a, const spVector b)
{
    return spVectorConstruct(a.x-b.x, a.y-b.y);
}

/// multiply a vector by a scalar value
inline spVector spMult(const spVector a, const spFloat b)
{
    return spVectorConstruct(a.x*b, a.y*b);
}

/// multiply a vector by a scalar value
inline spVector spMult(const spFloat a, const spVector b)
{
    return spMult(b, a);
}

inline spVector spRotate(const spVector a, const spVector b)
{
    return spVectorConstruct(a.x*b.x - a.y*b.y, a.x*b.y + a.y*b.x);
}

/// normalize a vector
inline void spNormalize(spVector* a)
{
    *a = spMult(*a, 1.0f / spLength(*a) + SP_FLT_EPSILON);
}

/// return a normalized vector
inline spVector spNormal(spVector a)
{
    spNormalize(&a);
    return a;
}

/// returns the distance squared between two vectors
inline spFloat spDistanceSquared(const spVector a, const spVector b)
{
    return spLengthSquared(spSub(a, b));
}

/// returns the distance between two vectors
inline spFloat spDistance(const spVector a, const spVector b)
{
    return spLength(spSub(a, b));
}

/// cross two vectors
inline spFloat spCross(const spVector a, const spVector b)
{
    return a.x*b.y - a.y*b.x;
}

/// cross a vector and a scalar
inline spVector spCross(const spVector a, const spFloat b)
{
    return spVectorConstruct(b*a.y, -b*a.x);
}

/// cross a vector and a scalar
inline spVector spCross(const spFloat a, const spVector b)
{
    return spVectorConstruct(-a*b.y, a*b.x);
}

/// negate a vector
inline void spNegate(spVector* a)
{
    a->x = -a->x;
    a->y = -a->y;
}

inline spVector spNegative(spVector a)
{
    spNegate(&a);
    return a;
}

inline void spVectorLog(spVector* vector, spInt8* msg = "")
{
    spLog("%0.7f, %0.7f, %s \n", vector->x, vector->y, msg);
}

/// @}

/// @ingroup spMatrix
/// @{

/// stack constructor
inline spMatrix spMatrixConstruct(const spFloat a, const spFloat b, const spFloat c, const spFloat d)
{
    spMatrix matrix;
    matrix.a = a;
    matrix.b = b;
    matrix.c = c;
    matrix.d = d;
    return matrix;
}

/// matrix setter
inline void spMatrixSet(spMatrix* m, const spFloat a, const spFloat b, const spFloat c, const spFloat d)
{
    m->a = a;  m->b = b;
    m->c = c;  m->d = d;
}

/// check for absolute equality between two matrix2's
inline spBool spEqual(const spMatrix a, const spMatrix b)
{
    return a.a == b.a && a.b == b.b && a.c == b.c && a.d == b.d;
}

/// add two matrices together component wise
inline spMatrix spAdd(const spMatrix a, const spMatrix b)
{
    return spMatrixConstruct(a.a+b.a, a.b+b.b, a.c+b.c, a.d+b.d);
}

/// sub two matrices together component wise
inline spMatrix spSub(const spMatrix a, const spMatrix b)
{
    return spMatrixConstruct(a.a-b.a, a.b-b.b, a.c-b.c, a.d-b.d);
}

/// multiply two matrix2's together
inline spMatrix spMult(const spMatrix a, const spMatrix b)
{
    return spMatrixConstruct(a.a*b.a + a.b*b.c,  a.a*b.b + a.b*b.d,
                    a.c*b.a + a.d*b.c,  a.c*b.b + a.d*b.d);
}

/// transform a vector by a matrix
inline spVector spMult(const spMatrix a, const spVector b)
{
    return spVectorConstruct(a.a*b.x + a.b*b.y, a.c*b.x + a.d*b.y);
}

/// multiply a matrix by a scalar
inline spMatrix spMult(const spMatrix a, const spFloat b)
{
    return spMatrixConstruct(a.a*b, a.b*b, a.c*b, a.d*b);
}

/// multiply a matrix by a scalar
inline spMatrix spMult(const spFloat b, const spMatrix a)
{
    return spMult(b, a);
}

/// multiply the transpose of a matrix times a matrix
inline spMatrix spTMult(const spMatrix a, const spMatrix b)
{
    return spMatrixConstruct(a.a*b.a + a.c*b.c, a.a*b.b + a.c*b.d,
                    a.b*b.a + a.d*b.c, a.b*b.b + a.d*b.d);
}

/// multiply a matrix by the trapose of a matrix
inline spMatrix spMultT(const spMatrix a, const spMatrix b)
{
    return spMatrixConstruct(a.a*b.a + a.b*b.b, a.a*b.c + a.b*b.d,
                    a.c*b.a + a.d*b.b, a.c*b.c + a.d*b.d);
}

/// multiply the transpose of two matrices together
inline spMatrix spTMultT(const spMatrix a, const spMatrix b)
{
    return spMatrixConstruct(a.a*b.a + a.c*b.b, a.a*b.c + a.c*b.d,
                    a.b*b.a + a.d*b.b, a.b*b.c + a.d*b.d);
}

/// divide a matrix by a scalar
inline spMatrix spDiv(const spMatrix a, const spFloat b)
{
    spFloat ib = 1.0f / (b + SP_FLT_EPSILON);
    return spMatrixConstruct(a.a*ib, a.b*ib, a.c*ib, a.d*ib);
}

/// get a specified axis from the matrix
inline spVector spGetAxis(const spMatrix a, spAxis axis)
{
    switch (axis)
    {
    case spAxis::x:
        return spVectorConstruct(a.a, a.c);
    case spAxis::y:
        return spVectorConstruct(a.b, a.d);
    }
    spAssert(false, "Invalid axis specification!");
}

/// get the determinant of the matrix
inline spFloat spDeterminant(const spMatrix a)
{
    return a.a * a.d - a.b * a.c;
}

/// get the transpose of a matrix
inline spMatrix spTranspose(const spMatrix a)
{
    return spMatrixConstruct(a.a, a.c, a.b, a.d);
}

/// transpose a matrix
inline void spTranspose(spMatrix* a)
{
    spFloat tmp = a->b;
    a->b = a->c;
    a->c = tmp;
}

/// get the inverse of a matrix
inline spMatrix spInverse(const spMatrix a)
{
    spFloat d = spDeterminant(a);
    if (d == 0.0f) return spMatrixZero();
    spFloat d_inv =  1.0f / d;
    return spMatrixConstruct(+a.d*d_inv, -a.b*d_inv, -a.c*d_inv, +a.a*d_inv);
}

/// invert a matrix2
inline void spInvert(spMatrix* a)
{
    spFloat d_inv = 1.0f / spDeterminant(*a) + SP_FLT_EPSILON;
    spFloat tmp;
    tmp =  + a->a * d_inv;
    a->a = + a->d * d_inv;
    a->b = - a->b * d_inv;
    a->c = - a->c * d_inv;
    a->d = tmp;
}

/// @}

/// @ingroup spRotation
/// @{

/// convert degrees to radians
inline spFloat spDeg2Rad(spFloat val)
{
    return val * SP_DEG_TO_RAD;
}

/// convert radians to degrees
inline spFloat spRad2Deg(spFloat val)
{
    return val * SP_DEG_TO_RAD;
}

inline spFloat spDegrees(const spFloat radians)
{
    return spRad2Deg(radians);
}

/// gets the angle of a rotation
inline spFloat spRotationGetAngle(const spRotation a)
{
    return spatan2(a.s, a.c);
}

/// gets the angle of a rotation in degrees
inline spFloat spRotationGetAngleDeg(const spRotation a)
{
    return spDegrees(spatan2(a.s, a.c));
}

/// sets the angle of a rotation in radians
inline void spRotationSet(spRotation* r, const spFloat a)
{
    r->s = spsin(a);
    r->c = spcos(a);
}

/// sets the cos and sin in radians of a rotation
inline void spRotationSet(spRotation* r, const spFloat s, const spFloat c)
{
    r->s = s;
    r->c = c;
}

/// stack constructor
inline spRotation spRotationConstruct(const spFloat angle)
{
    spRotation rot;
    spRotationSet(&rot, angle);
    return rot;
}

/// stack constructor in radians
inline spRotation spRotationRadiansConstruct(const spFloat s, const spFloat c)
{
    spRotation rot;
    spRotationSet(&rot, s, c);
    return rot;
}

/// transform a vector by a rotation
inline spVector spMult(const spRotation a, const spVector b)
{
    // mod
    return spVectorConstruct(a.c*b.x - a.s*b.y, a.s*b.x + a.c*b.y);
}

/// combine two rotations
inline spRotation spMult(const spRotation a, const spRotation b)
{
    return spRotationRadiansConstruct(a.s*b.c + a.c*b.s, a.c*b.c - a.s*b.s);
}

/// inverse rotate a vector
inline spVector spTMult(const spRotation a, const spVector b)
{
    /// mod
    return spVectorConstruct(a.c*b.x + a.s*b.y, -a.s*b.x + a.c*b.y);
}

/// inverse rotate another rotation
inline spRotation spTMult(const spRotation a, const spRotation b)
{
    return spRotationRadiansConstruct(a.c*b.s - a.s*b.c, a.c*b.c + a.s*b.s);
}

/// @}

/// @ingroup spTransform
/// @{

/// stack constructor
inline spTransform spTransformConstruct(const spVector p, const spRotation q)
{
    return { p, q };
}

/// setter for transforms
inline spTransform spTransformSet(spTransform* a, const spVector p, const spRotation q)
{
    a->p = p;
    a->q = q;
}

/// transform a vector
inline spVector spMult(const spTransform a, const spVector b)
{
    return spAdd(spMult(a.q, b), a.p);
}

/// transform another transform. put it into a's coordinate system
inline spTransform spMult(const spTransform a, const spTransform b)
{
    return spTransformConstruct(spAdd(spMult(a.q, b.p), a.p), spMult(a.q, b.q));
}

/// inverse transform a vector
inline spVector spTMult(const spTransform a, const spVector b)
{
    return spTMult(a.q, spSub(b, a.p));
}

/// inverse transform another transform. removes it froms a's coordinate system
inline spTransform spTMult(const spTransform a, const spTransform b)
{
    return spTransformConstruct(spTMult(a.q, spSub(b.p, a.p)), spTMult(a.q, b.q));
}

/// @}

/// linearly interpolate between two floats
inline spFloat spLerp(spFloat a, spFloat b, spFloat t)
{
    /// slightly faster than a * (1.0f - f) + (b * f). can be less precisem i will change later if needed.
    return a + t * (b - a);
}

/// linearly interpolate between two vectors
inline spVector spLerp(const spVector a, const spVector b, spFloat t)
{
    return spAdd(a, spMult(t, spSub(b, a)));
}

/// cubic hermite interpolation on floats
inline spFloat spSmoothstep(spFloat a, spFloat b, spFloat t)
{
    if (t < a) return a;
    if (t > b) return b;

    spFloat t0 = (t - a) / (b - a);
    return t0 * t0 * (3.0f - 2.0f * t0);
}

/// TODO:
inline spBool spAlmostEqual(const spVector a, const spVector b, const spFloat EPSILON = SP_FLT_EPSILON)
{
    return spAlmostEqual(a.x, b.x, EPSILON) && spAlmostEqual(a.y, b.y, EPSILON);
}

/// @ingroup spDebug spDebug
/// @{

#ifdef SP_DEBUG
    #define VECCHECK(vec) NANCHECK(vec.x); NANCHECK(vec.y)
    #define ROTCHECK(rot) NANCHECK(rot.p); NANCHECK(rot.q)
    #define MATCHECK(mat) NANCHECK(mat.a); NANCHECK(mat.b);\
                          NANCHECK(mat.c); NANCHECK(mat.d)
    #define XFCHECK(xf)   ROTCHECK(xf.q);  VECCHECK(xf.p)
#else
    #define VECCHECK(vec)
    #define ROTCHECK(rot)
    #define MATCHECK(mat)
    #define XFCHECK(xf)
#endif

/// @}

#endif