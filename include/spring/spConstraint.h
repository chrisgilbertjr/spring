
#ifndef SP_CONSTRAINT_H
#define SP_CONSTRAINT_H

#include "spMath.h"

/// @defgroup spConstraint spConstraint
/// @{

/// baumgarte velocity bias coef
extern spFloat spBaumgarte;

/// different joint types
typedef enum 
{
    SP_ANGULAR_SPRING_JOINT = 0,
    SP_DISTANCE_JOINT = 1,
    SP_SPRING_JOINT = 2,
    SP_MOTOR_JOINT = 3,
    SP_WHEEL_JOINT = 4,
    SP_POINT_JOINT = 5,
    SP_MOUSE_JOINT = 6,
    SP_ROPE_JOINT = 7,
    SP_GEAR_JOINT = 8,
    SP_JOINT_SIZE
} spConstraintType;

typedef void (*spFreeFunc)(spConstraint* constraint);
typedef void (*spPreSolveFunc)(spConstraint* constraint, const spFloat h);
typedef void (*spWarmStartFunc)(spConstraint* constraint);
typedef void (*spSolveFunc)(spConstraint* constraint);

/// constraint function pointers
struct spConstraintFuncs
{
    spFreeFunc      free;      ///< constraint free func
    spPreSolveFunc  preSolve;  ///< constraint pre solve func
    spWarmStartFunc warmStart; ///< warm start func
    spSolveFunc     solve;     ///< solve func
};

/// a constraint is a limit put between two bodies
/// check the joints for more detailed descriptions of what limits they place on bodies
struct spConstraint
{
    spConstraintFuncs funcs; ///< constraint functions (solve, free, etc...)
    spConstraintType type;   ///< the joint type
    spConstraint* next;      ///< the next constraint in the doubly linked list
    spConstraint* prev;      ///< the prev constraint in the doubly linked list
    spBody* bodyA;           ///< the first body of the joint
    spBody* bodyB;           ///< the second body of the joint
    spWorld* world;          ///< world the constraint is in
};

INLINE void spConstraintInitFuncs(spConstraintFuncs* funcs, spFreeFunc free, spPreSolveFunc preSolve, spWarmStartFunc warmStart, spSolveFunc solve)
{
    funcs->free = free;
    funcs->preSolve = preSolve;
    funcs->warmStart = warmStart;
    funcs->solve = solve;
}

/// initialize a constraint given two bodies and a type
SPRING_API void spConstraintInit(spConstraint* constraint, struct spBody* a, struct spBody* b, spConstraintType type); 

/// free a constraint from the heap
SPRING_API void spConstraintFree(spConstraint* constraint);

/// construct a constraint on the stack given two bodies and a type
SPRING_API spConstraint spConstraintConstruct(struct spBody* a, struct spBody* b, spConstraintType type);

/// get the next constraint in the list
SPRING_API spConstraint* spConstraintGetNext(spConstraint* constraint);

/// get the prev constraint in the list
SPRING_API spConstraint* spConstraintGetPrev(spConstraint* constraint);

/// get the constraints first body
SPRING_API spBody* spConstraintGetBodyA(spConstraint* constraint);

/// get the constraints second body
SPRING_API spBody* spConstraintGetBodyB(spConstraint* constraint);

/// get the constraints type
SPRING_API spConstraintType spConstraintGetType(spConstraint* constraint);

/// @}

#endif