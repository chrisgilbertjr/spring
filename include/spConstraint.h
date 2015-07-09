
#ifndef SP_CONSTRAINT_H
#define SP_CONSTRAINT_H

#include "spCore.h"

/// @defgroup spConstraint spConstraint
/// @{

/// baumgarte velocity bias coef
extern spFloat spBaumgarte;

/// different joint types
enum spConstraintType
{
    SP_ANGULAR_SPRING_JOINT,
    SP_DISTANCE_JOINT,
    SP_SPRING_JOINT,
    SP_MOTOR_JOINT,
    SP_WHEEL_JOINT,
    SP_POINT_JOINT,
    SP_MOUSE_JOINT,
    SP_ROPE_JOINT,
    SP_GEAR_JOINT,
    SP_JOINT_SIZE
};

/// a constraint is a limit put between two bodies
/// check the joints for more detailed descriptions of what limits they place on bodies
struct spConstraint
{
    spConstraintType type; ///< the joint type
    spConstraint* next;    ///< the next constraint in the doubly linked list
    spConstraint* prev;    ///< the prev constraint in the doubly linked list
    spBody* bodyA;         ///< the first body of the joint
    spBody* bodyB;         ///< the second body of the joint
};

/// initialize a constraint given two bodies and a type
void spConstraintInit(spConstraint* constraint, struct spBody* a, struct spBody* b, spConstraintType type); 

/// free a constraint from the heap
void spConstraintFree(spConstraint* constraint);

/// construct a constraint on the stack given two bodies and a type
spConstraint spConstraintConstruct(struct spBody* a, struct spBody* b, spConstraintType type);

/// warm start the constraint given the timestep
void spConstraintApplyCachedImpulse(spConstraint* constraint);

/// setup the constraint to be solved
void spConstraintPreSolve(spConstraint* constraint, const spFloat h);

/// solve the constraint and apply the impulse
void spConstraintSolve(spConstraint* constraint);

/// get the next constraint in the list
spConstraint* spConstraintGetNext(spConstraint* constraint);

/// get the prev constraint in the list
spConstraint* spConstraintGetPrev(spConstraint* constraint);

/// get the constraints first body
struct spBody* spConstraintGetBodyA(spConstraint* constraint);

/// get the constraints second body
struct spBody* spConstraintGetBodyB(spConstraint* constraint);

/// get the constraints type
spConstraintType spConstraintGetType(spConstraint* constraint);

/// @}

#endif