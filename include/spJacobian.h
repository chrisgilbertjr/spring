
#ifndef SP_JACOBIAN_H
#define SP_JACOBIAN_H

#include "spMath.h"

/// @defgroup spJacobian spJacobian
/// @{

/// J = [ vA, wA, vB, wB ]
struct spJacobian
{
    spVector vA;
    spVector vB;
    spFloat  wA;
    spFloat  wB;
};

void spJacobianInit(spJacobian* j, const spVector& vA, const spVector& vb, const spFloat wA, const spFloat wB);

spJacobian spJacobianConstruct(const spVector& vA, const spVector& vb, const spFloat wA, const spFloat wB);

void spMult(spJacobian* r, const spFloat s, const spJacobian& j);

void spMult(spJacobian* r, const spJacobian& j, const spFloat s);

void spMultT(spJacobian* r, const spFloat s, const spJacobian& j);

void spMultT(spJacobian* r, const spJacobian& j, const spFloat s);

/// @}

#endif