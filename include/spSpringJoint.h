
#ifndef SP_SPRING_JOINT_H
#define SP_SPRING_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

/// a spring joint is a soft distance constraint
/// the spring parameters are constrolled by the frequency and damping parameters
/// frequency is oscilations per second, while damping represents its velocity
/// loss during the simulation. a value of 0 damping with oscilate forever,
/// while a value of 1 will asscilate and stop very quickly
///
/// constraints:
/// C    = normal(pA - pB) - restLength
/// Cdot = dot(n, vB + cross(wB, rB) - vA - cross(wA, rA))
/// J    = [-n  -cross(rA, n)  n  cross(rB, n)]
/// K    = invMassA + invInertA * cross(rA, n)^2 + invMassB + invInertB * cross(rB, n)^2
struct spSpringJoint
{
    spConstraint constraint; /// INTERNAL: base class
    spVector anchorA;        ///           local anchor on body a
    spVector anchorB;        ///           local anchor on body b
    spVector rA;             /// INTERNAL: relative velocity of anchor a to com of body a
    spVector rB;             /// INTERNAL: relative velocity of anchor b to com of body b
    spVector n;              /// INTERNAL: normal impulse direction
    spFloat lambdaAccum;     /// INTERNAL: accumulated impulse magnitude
    spFloat restLength;      ///           rest length of the spring
    spFloat frequency;       ///           frequency of oscilation per second
    spFloat damping;         ///           springs damping
    spFloat eMass;           /// INTERNAL: effective mass of the joint = K
    spFloat gamma;           /// INTERNAL: feeds lambda back into velocity constraint. used for soft constraint
    spFloat beta;            /// INTERNAL: feeds position error back into velocity constraint. used for soft constraint
};

/// gets anchor a in world coordinates of body a
spVector spSpringJointGetWorldAnchorA(spSpringJoint* joint);

/// gets anchor b in world coordinates of body b
spVector spSpringJointGetWorldAnchorB(spSpringJoint* joint);

/// sets anchor a in world coordinates of body a
void spSpringJointSetWorldAnchorA(spSpringJoint* joint, spVector anchorA);

/// sets anchor b in world coordinates of body b
void spSpringJointSetWorldAnchorB(spSpringJoint* joint, spVector anchorB);

/// init a spring joint with 2 bodies, 2 anchor points, damping, frequency, and damping.
void spSpringJointInit(spSpringJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat frequency, spFloat damping, spFloat restLength);

/// allocate a spring joint on the heap
spSpringJoint* spSpringJointAlloc();

/// create a spring joint on the heap with 2 bodies, 2 anchor points, damping, frequency, and damping.
spSpringJoint* spSpringJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat frequency, spFloat damping, spFloat restLength);

/// release the memory pointed at by a spring joint
void spSpringJointFree(spSpringJoint** joint);

/// setup a spring joint to be solved by the solver
void spSpringJointPreStep(spSpringJoint* joint, const spFloat h);

/// solve and apply impulses to a spring joint 
void spSpringJointSolve(spSpringJoint* joint);

/// gets anchor a in local coordinates of body a
inline spVector spSpringJointGetAnchorA(spSpringJoint* joint) { return joint->anchorA; };

/// gets anchor b in local coordinates of body b
inline spVector spSpringJointGetAnchorB(spSpringJoint* joint) { return joint->anchorB; };

/// gets the length of the impulse that was applied last timestep
inline spFloat spSpringJointGetImpulseMag(spSpringJoint* joint) { return joint->lambdaAccum; };

/// gets the rest length of the spring
inline spFloat spSpringJointGetRestLength(spSpringJoint* joint) { return joint->restLength; };

/// gets the oscilations per second of the spring
inline spFloat spSpringJointGetFrequency(spSpringJoint* joint) { return joint->frequency; };

/// gets the damping coefficient of a spring
inline spFloat spSpringJointGetDamping(spSpringJoint* joint) { return joint->damping; };

/// sets anchor points a in local coordinates of body a
inline void spSpringJointSetAnchorA(spSpringJoint* joint, spVector anchorA) { joint->anchorA = anchorA; };

/// sets anchor points b in local coordinates of body b
inline void spSpringJointSetAnchorB(spSpringJoint* joint, spVector anchorB) { joint->anchorB = anchorB; };

/// sets the rest length of the spring
inline void spSpringJointSetRestLength(spSpringJoint* joint, spFloat restLength) { joint->restLength = restLength; };

/// sets the oscilation per second of the spring 
inline void spSpringJointSetFrequency(spSpringJoint* joint, spFloat frequency) { joint->frequency = frequency; };

/// sets the damping coefficient of the spring
inline void spSpringJointSetDamping(spSpringJoint* joint, spFloat damping) { joint->damping = damping; };

#endif