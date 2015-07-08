
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
        spRopeJointApplyCachedImpulse((spRopeJoint*) constraint);
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
        spGearJointApplyCachedImpulses((spGearJoint*) constraint);
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
        spDistanceJointPreSolve((spDistanceJoint*) constraint, h);
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