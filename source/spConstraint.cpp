
#include "spAngularSpringJoint.h"
#include "spDistanceJoint.h"
#include "spSpringJoint.h"
#include "spPointJoint.h"
#include "spMotorJoint.h"
#include "spWheelJoint.h"
#include "spMouseJoint.h"
#include "spRopeJoint.h"
#include "spGearJoint.h"

spFloat spBaumgarte = 0.2f;

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
spConstraintFree(spConstraint* constraint)
{
    /// TODO: add function pointers to constraints so we can get rid of these ugly if else statements...
    if (constraint->type == SP_DISTANCE_JOINT)
    {
        spDistanceJoint* distance = (spDistanceJoint*) constraint;
        spDistanceJointFree(&distance);
    }
    else if (constraint->type == SP_ROPE_JOINT)
    {
        spRopeJoint* rope = (spRopeJoint*) constraint;
        spRopeJointFree(&rope);
    }
    else if (constraint->type == SP_MOTOR_JOINT)
    {
        spMotorJoint* motor = (spMotorJoint*) constraint;
        spMotorJointFree(&motor);
    }
    else if (constraint->type == SP_SPRING_JOINT)
    {
        spSpringJoint* spring = (spSpringJoint*) constraint;
        spSpringJointFree(&spring);
    }
    else if (constraint->type == SP_ANGULAR_SPRING_JOINT)
    {
        spAngularSpringJoint* angularSpring = (spAngularSpringJoint*) constraint;
        spAngularSpringJointFree(&angularSpring);
    }
    else if (constraint->type == SP_WHEEL_JOINT)
    {
        spWheelJoint* wheel = (spWheelJoint*) constraint;
        spWheelJointFree(&wheel);
    }
    else if (constraint->type == SP_GEAR_JOINT)
    {
        spGearJoint* gear = (spGearJoint*) constraint;
        spGearJointFree(&gear);
    }
    else if (constraint->type == SP_POINT_JOINT)
    {
        spPointJoint* point = (spPointJoint*) constraint;
        spPointJointFree(&point);
    }
    else if (constraint->type == SP_MOUSE_JOINT)
    {
        spMouseJoint* mouse = (spMouseJoint*) constraint;
    }
}

void 
spConstraintApplyCachedImpulse(spConstraint* constraint)
{
    /// TODO: add function pointers to constraints so we can get rid of these ugly switch statment...
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointApplyCachedImpulse((spDistanceJoint*) constraint);
        break;
    case SP_ROPE_JOINT:
        spRopeJointApplyCachedImpulse((spRopeJoint*) constraint);
        break;
    case SP_MOTOR_JOINT:
        spMotorJointApplyCachedImpulse((spMotorJoint*) constraint);
        break;
    case SP_SPRING_JOINT:
        spSpringJointApplyCachedImpulse((spSpringJoint*) constraint);
        break;
    case SP_ANGULAR_SPRING_JOINT:
        spAngularSpringJointApplyCachedImpulse((spAngularSpringJoint*) constraint);
        break;
    case SP_WHEEL_JOINT:
        spWheelJointApplyCachedImpulse((spWheelJoint*) constraint);
        break;
    case SP_GEAR_JOINT:
        spGearJointApplyCachedImpulse((spGearJoint*) constraint);
        break;
    case SP_POINT_JOINT:
        spPointJointApplyCachedImpulse((spPointJoint*) constraint);
        break;
    case SP_MOUSE_JOINT:
        /// no need.
        break;
    default:
        spAssert(false, "constraint type is not valid in prestep!\n");
    }
}

void 
spConstraintPreSolve(spConstraint* constraint, const spFloat h)
{
    /// TODO: add function pointers to constraints so we can get rid of these ugly switch statment...
    switch (constraint->type)
    {
    case SP_DISTANCE_JOINT:
        spDistanceJointPreSolve((spDistanceJoint*) constraint, h);
        break;
    case SP_ROPE_JOINT:
        spRopeJointPreSolve((spRopeJoint*) constraint, h);
        break;
    case SP_MOTOR_JOINT:
        spMotorJointPreSolve((spMotorJoint*) constraint, h);
        break;
    case SP_SPRING_JOINT:
        spSpringJointPreSolve((spSpringJoint*) constraint, h);
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
    /// TODO: add function pointers to constraints so we can get rid of these ugly switch statment...
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

spConstraint* 
spConstraintGetNext(spConstraint* constraint)
{
    return constraint->next;
}

spConstraint* 
spConstraintGetPrev(spConstraint* constraint)
{
    return constraint->prev;
}

struct spBody* 
spConstraintGetBodyA(spConstraint* constraint)
{
    return constraint->bodyA;
}

struct spBody* 
spConstraintGetBodyB(spConstraint* constraint)
{
    return constraint->bodyB;
}

spConstraintType 
spConstraintGetType(spConstraint* constraint)
{
    return constraint->type;
}