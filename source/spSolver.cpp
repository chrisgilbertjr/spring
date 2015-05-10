
#include "spSolver.h"
#include "spMath.h"

spFloat 
spConstraintMass(
    const spFloat mass_inv1,
    const spFloat mass_inv2,
    const spFloat i_inv1,
    const spFloat i_inv2,
    const spVector r1,
    const spVector r2,
    const spVector norm)
{
    return 0.0f;
}

spVector 
spRelativeVelocity(
    const spVector& vA, 
    const spVector& vB, 
    const spFloat   wA, 
    const spFloat   wB,
    const spVector& rA, 
    const spVector& rB)
{
    spVector v1 = spAdd(vA, spMult(spSkew(rA), wA));
    spVector v2 = spAdd(vB, spMult(spSkew(rB), wB));
    return spSub(v2, v1);
}

spVector 
tang_impulse(
    const spVector& rv, 
    const spVector* tang, 
    const spFloat& u, 
    const spVector& P_norm, 
    const spFloat k_tang)
{
    return spVector(0,0);
}

spVector 
norm_impulse(
    const spVector& rv, 
    const spVector& norm, 
    const spVector& bias, 
    const spFloat k_norm)
{
    return spVector(0,0);
}

spFloat 
bias_velocity(const spFloat slop, const spFloat bias, const spFloat h)
{
    return 0.0f;
}

spFloat 
tang_relative_velocity(const spVector& rv, const spVector& norm)
{
    return 0.0f;
}

spVector 
norm_relative_velocity(const spVector& rv, const spVector& norm)
{
    return spVector(0,0);
}