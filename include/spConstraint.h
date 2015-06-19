
#ifndef SP_CONSTRAINT_H
#define SP_CONSTRAINT_H

#include "spCore.h"

/// @defgroup spConstraint constraint base class for joints
/// @{

/// Constraints are limitations of one or more degrees of freedom of a rigid body.
/// Constraints are also pairwise, and put these limitations on a pair of rigid bodies
///
/// Constraints have a general form: C(lA, aA, lB, aB) = 0
/// This is a "position constraint".
///
/// An example of a position constraint is the distance constraint.
/// ex: Distance constraint  C(x, y) = |p2 - p1| - D = 0. 
/// This constraint says that the distance of two points should always be D length away.
///
/// After modeling the position constraint, you differentiate to get the velocity constraint.
/// .
/// C(lA, aA, lB, aB) = JV = 0
///
/// After differentiating for the distance constraint you get:
/// .
/// C = (D dot vB + wB dot rB x D) - (D dot vA + wA dot rA x D)
/// By inspection:
/// J = [ -D, -rA x D, D, rB x D ]
///
/// We solve each velocity constraint for a lagrange multiplier (lamba) to find the impulse magnitude.
/// We can use this magnitude and the lagrange multiplier to compute a constraint impulse.
///
/// L = -(J*V-b) / E
/// E = J*Mi*Jt
/// F = L * Jt
///
/// t  = transpose
/// i  = inverse
/// E  = effective mass
/// L  = lagrange multiplier (lamda)
/// F  = constraint force (impulse)
/// J  = jacobian   = [ lA, aA, lB, aB ] - each jacobian is constraint dependent and is derived by hand
/// Vt = velocities = [ vA, wA, vB, wB ]
/// Mi = constraint mass = |miA, 0,   0,   0|
///                        |0, iiA,   0,   0|
///                        |0,   0, miB,   0|
///                        |0,   0,   0, iiB|
/// A = rigid body A
/// B = rigid body B
/// l = jacobians linear velocity - (computed from derived jacobian vector)
/// a = jacobians angular velocity - (computed from derived jacobian vector)
/// v = linear velocity
/// w = angular velocity
/// mi = inv mass
/// ii = inv inertia
///
/// Once L (lambda) is found, you can then update the velocities of each body
///
/// V += Mi * F
///
/// Even though Mi is a matrix, you should never model it as one since most entries are 0.
/// Instead, to apply the impulse you can simplify it to this:
///
/// vA += miA * lA;
/// wA += iiA * aA;
/// vB += miB * lB;
/// wB += iiB * aB;

struct spBody;

enum spConstraintType
{
    SP_DISTANCE_JOINT,
    SP_MOTOR_JOINT,
    SP_ROPE_JOINT,
    SP_SPRING_JOINT,
    SP_ANGULAR_SPRING_JOINT,
    SP_WHEEL_JOINT,
    SP_GEAR_JOINT,
    SP_POINT_JOINT,
};

struct spConstraint
{
    spConstraintType type;
    spConstraint* next;
    spConstraint* prev;
    spBody* body_a;
    spBody* body_b;
};

void spConstraintInit(spConstraint* constraint, spBody* a, spBody* b, spConstraintType type);

spConstraint spConstraintConstruct(spBody* a, spBody* b, spConstraintType type);

void spConstraintPreStep(spConstraint* constraint, const spFloat h);

void spConstraintSolve(spConstraint* constraint);

void spConstraintStabilize(spConstraint* constraint);

/// @}

#endif