
#ifndef SP_ROPE_JOINT_H
#define SP_ROPE_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

/// @defgroup spRopeJoint spRopeJoint
/// @{

/// a rope joint is a hard constraint with a max distance between two anchor points
struct spRopeJoint
{
    spConstraint constraint; ///< base constraint class
    spVector anchorA;        ///< anchor point in body A's local space
    spVector anchorB;        ///< anchor point in body B's local space
    spVector rA;             ///< rel velocity from anchorA to body A's com
    spVector rB;             ///< rel velocity from anchorB to body B's com
    spVector n;              ///< norm direction of rope joint constraint
    spFloat maxDistance;     ///< max distance the two anchor points can be from one another
    spFloat lambdaAccum;     ///< accumulated lagrange multiplier
    spFloat eMass;           ///< effective mass (K)
    spFloat bias;            ///< baumgarte bias velocity
};

/// initialize a rope joint with two bodies, two anchor points in local space, and a max distance
void spRopeJointInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance);

/// initialize a rope joint with two bodies, two anchor points in world space, and a max distance
void spRopeJointWorldInit(spRopeJoint* joint, spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance);

/// allocate a rope joint on the heap
spRopeJoint* spRopeJointAlloc();

/// allocate a rope joint on the heap, and init it with two bodies, two anchor points in local space, and a max distance
spRopeJoint* spRopeJointNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance);

/// allocate a rope joint on the heap, and init it with two bodies, two anchor points in world space, and a max distance
spRopeJoint* spRopeJointWorldNew(spBody* a, spBody* b, spVector anchorA, spVector anchorB, spFloat maxDistance);

/// free memory pointed at by a rope joint
void spRopeJointFree(spRopeJoint** joint);

/// setup a rope joint to be solved
void spRopeJointPreSolve(spRopeJoint* joint, const spFloat h);

/// apply the last timesteps impulse, for a more stable simulation
void spRopeJointApplyCachedImpulse(spRopeJoint* joint);

/// solve the rope joint, and apply impulses to the bodies
void spRopeJointSolve(spRopeJoint* joint);

/// check if a constraint is a rope joint
spBool spConstraintIsRopeJoint(spConstraint* constraint);

/// safely cast a constraint to a rope joint if its that type
spRopeJoint* spConstraintCastRopeJoint(spConstraint* constraint);

/// get anchorA in A's local space
spVector spRopeJointGetlAnchorA(spConstraint* constraint);

/// get anchorB in B's local space
spVector spRopeJointGetAnchorB(spConstraint* constraint);

/// get anchorA in world space
spVector spRopeJointGetWorldAnchorA(spConstraint* constraint);

/// get anchorA in world space
spVector spRopeJointGetWorldAnchorB(spConstraint* constraint);

/// get the joints maximum distance
spFloat spRopeJointGetMaxDistance(spConstraint* constraint);

/// set body A's anchor point in A's local space
void spRopeJointSetAnchorA(spConstraint* constraint, spVector anchorA);

/// set body B's anchor point in B's local space
void spRopeJointSetAnchorB(spConstraint* constraint, spVector anchorB);

/// set body A's anchor point in world space
void spRopeJointSetWorldAnchorA(spConstraint* constraint, spVector anchorA);

/// set body B's anchor point in world space
void spRopeJointSetWorldAnchorB(spConstraint* constraint, spVector anchorB);

/// set the max distance between the joints two anchor points
void spRopeJointSetMaxDistance(spConstraint* constraint, spFloat maxDistance);

/// @}

#endif