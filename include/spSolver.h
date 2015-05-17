
#ifndef SP_SOLVER_H
#define SP_SOLVER_H

#include "spCore.h"

/// @defgroup spSolver constraint solver helper functions
/// @{

spFloat spConstraintMass(
    const spFloat mass_inv1,
    const spFloat mass_inv2,
    const spFloat i_inv1,
    const spFloat i_inv2,
    const spVector r1,
    const spVector r2,
    const spVector norm);

spVector spRelativeVelocity(
    const spVector& vA, 
    const spVector& vB, 
    const spFloat   wA, 
    const spFloat   wB,
    const spVector& rA, 
    const spVector& rB);

spVector tang_impulse(
    const spVector& rv, 
    const spVector* tang, 
    const spFloat& u, 
    const spVector& P_norm, 
    const spFloat k_tang);

spVector norm_impulse(
    const spVector& rv, 
    const spVector& norm, 
    const spVector& bias, 
    const spFloat k_norm);

spFloat bias_velocity(const spFloat slop, const spFloat bias, const spFloat h);

spFloat tang_relative_velocity(const spVector& rv, const spVector& norm);

spVector norm_relative_velocity(const spVector& rv, const spVector& norm);


/// @}

#endif