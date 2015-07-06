
#include "spAngularSpringJoint.h"
#include "spDistanceJoint.h"
#include "spSpringJoint.h"
#include "spPointJoint.h"
#include "spMotorJoint.h"
#include "spWheelJoint.h"
#include "spMouseJoint.h"
#include "spRopeJoint.h"
#include "spGearJoint.h"

void 
spConstraintInit(spConstraint* constraint, spBody* a, spBody* b, spConstraintType type)
{
    NULLCHECK(constraint);
    constraint->type = type;
    constraint->next = NULL;
    constraint->prev = NULL;
    constraint->bodyA = a;
    constraint->bodyB = b;
}

spConstraint
spConstraintConstruct(spBody* a, spBody* b, spConstraintType type)
{
    spConstraint constraint;
    spConstraintInit(&constraint, a, b, type);
    return constraint;
}

void 
spConstraintApplyCachedImpulse(spConstraint* constraint, const spFloat h)
{
    NULLCHECK(constraint);
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        break;
    case SP_ROPE_JOINT:
        spRopeJointApplyCachedImpulse((spRopeJoint*) constraint, h);
        break;
    case SP_MOTOR_JOINT:
        break;
    case SP_SPRING_JOINT:
        break;
    case SP_ANGULAR_SPRING_JOINT:
        break;
    case SP_WHEEL_JOINT:
        break;
    case SP_GEAR_JOINT:
        break;
    case SP_POINT_JOINT:
        break;
    case SP_MOUSE_JOINT:
        break;
    default:
        spAssert(false, "constraint type is not valid in prestep!\n");
    }
}

void 
spConstraintPreSolve(spConstraint* constraint, const spFloat h)
{
    NULLCHECK(constraint);
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointPreStep((spDistanceJoint*) constraint, h);
        break;
    case SP_ROPE_JOINT:
        spRopeJointPreSolve((spRopeJoint*) constraint, h);
        break;
    case SP_MOTOR_JOINT:
        spMotorJointPreStep((spMotorJoint*) constraint, h);
        break;
    case SP_SPRING_JOINT:
        spSpringJointPreStep((spSpringJoint*) constraint, h);
        break;
    case SP_ANGULAR_SPRING_JOINT:
        spAngularSpringJointPreSolve((spAngularSpringJoint*) constraint, h);
        break;
    case SP_WHEEL_JOINT:
        spWheelJointPreSolve((spWheelJoint*) constraint, h);
        break;
    case SP_GEAR_JOINT:
        spGearJointPreSolve((spGearJoint*) constraint, h);
        break;
    case SP_POINT_JOINT:
        spPointJointPreSolve((spPointJoint*) constraint, h);
        break;
    case SP_MOUSE_JOINT:
        spMouseJointPreSolve((spMouseJoint*) constraint, h);
        break;
    default:
        spAssert(false, "constraint type is not valid in prestep!\n");
    }
}

void 
spConstraintSolve(spConstraint* constraint)
{
    NULLCHECK(constraint);
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointSolve((spDistanceJoint*) constraint);
        break;
    case SP_ROPE_JOINT:
        spRopeJointSolve((spRopeJoint*) constraint);
        break;
    case SP_MOTOR_JOINT:
        spMotorJointSolve((spMotorJoint*) constraint);
        break;
    case SP_SPRING_JOINT:
        spSpringJointSolve((spSpringJoint*) constraint);
        break;
    case SP_ANGULAR_SPRING_JOINT:
        spAngularSpringJointSolve((spAngularSpringJoint*) constraint);
        break;
    case SP_WHEEL_JOINT:
        spWheelJointSolve((spWheelJoint*) constraint);
        break;
    case SP_GEAR_JOINT:
        spGearJointSolve((spGearJoint*) constraint);
        break;
    case SP_POINT_JOINT:
        spPointJointSolve((spPointJoint*) constraint);
        break;
    case SP_MOUSE_JOINT:
        spMouseJointSolve((spMouseJoint*) constraint);
        break;
    default:
        spAssert(false, "constraint type is not valid in solve!\n");
    }
}

struct spAngularSpringJoint*
spConstraintCastAngularSpringJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_ANGULAR_SPRING_JOINT)
    {
        return (spAngularSpringJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not an angular spring joint\n");
        return NULL;
    }
}

struct spDistanceJoint*
spConstraintCastDistanceJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_DISTANCE_JOINT)
    {
        return (spDistanceJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a distance joint\n");
        return NULL;
    }
}

struct spSpringJoint*
spConstraintCastSpringJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_SPRING_JOINT)
    {
        return (spSpringJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a spring joint\n");
        return NULL;
    }
}

struct spPointJoint*
spConstraintCastPointJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_POINT_JOINT)
    {
        return (spPointJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a point joint\n");
        return NULL;
    }
}

struct spMotorJoint*
spConstraintCastMotorJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_MOTOR_JOINT)
    {
        return (spMotorJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a motor joint\n");
        return NULL;
    }
}

struct spWheelJoint*
spConstraintCastWheelJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_WHEEL_JOINT)
    {
        return (spWheelJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a wheel joint\n");
        return NULL;
    }
}

struct spMouseJoint*
spConstraintCastMouseJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_MOUSE_JOINT)
    {
        return (spMouseJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a mouse joint\n");
        return NULL;
    }
}

struct spRopeJoint*
spConstraintCastRopeJoint(spConstraint* constraint)
{
    NULLCHECK(constraint);
    if (constraint->type == SP_ROPE_JOINT)
    {
        return (spRopeJoint*) constraint;
    }
    else
    {
        spWarning(spFalse, "constraint is not a rope joint\n");
        return NULL;
    }
}

struct spBody* 
spConstraintGetBodyA(spConstraint* constraint)
{
    NULLCHECK(constraint);
    return constraint->bodyA;
}

struct spBody* 
spConstraintGetBodyB(spConstraint* constraint)
{
    NULLCHECK(constraint);
    return constraint->bodyB;
}

spConstraintType 
spConstraintGetType(spConstraint* constraint)
{
    NULLCHECK(constraint);
    return constraint->type;
}