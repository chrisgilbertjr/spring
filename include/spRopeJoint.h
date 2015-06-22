
#ifndef SP_ROPE_JOINT_H
#define SP_ROPE_JOINT_H

#include "spConstraint.h"
#include "spMath.h"

/// a rope joint is a hard constraint with a max distance between two anchor points
/// 
/// max distance constraint:
/// C    = length(anchorA - anchorB) - maxDist
/// n    = norm(anchorA - anchorB)
/// Cdot = dot(norm, (vB + cross(wB, rB)) - (vA + cross(wA, rA)))
/// J    = [ -n  -cross(rA, n)  n  cross(rB, n) ]
/// K    = 1.0 / miA + miB + iiA * cross(n, rA)^2 + iiB * cross(n, rB)^2
struct spRopeJoint
{
    spConstraint constraint; ///< INTERNAL: base constraint class
    spVector anchorA;        ///<         : anchor point in body A's local space
    spVector anchorB;        ///<         : anchor point in body B's local space
    spVector rA;             ///< INTERNAL: rel velocity from anchorA to body A's com
    spVector rB;             ///< INTERNAL: rel velocity from anchorB to body B's com
    spVector n;              ///< INTERNAL: norm direction of rope joint constraint
    spFloat maxDistance;     ///<         : max distance the two anchor points can be from one another
    spFloat lambdaAccum;     ///< INTERNAL: accumulated impulse magnitude
    spFloat eMass;           ///< INTERNAL: effective mass (K)
    spFloat bias;            ///< INTERNAL: baumgarte bias velocity
};

/// get anchorA in A's local space
spVector spRopeJointGetLocalAnchorA(spRopeJoint* joint);

/// get anchorB in B's local space
spVector spRopeJointGetLocalAnchorB(spRopeJoint* joint);

/// get anchorA in world space
spVector spRopeJointGetWorldAnchorA(spRopeJoint* joint);

/// get anchorA in world space
spVector spRopeJointGetWorldAnchorB(spRopeJoint* joint);

/// get the last timesteps impulse direction, the vector is normalized
spVector spRopeJointGetImpulseDirection(spRopeJoint* joint);

/// get the last timesteps impulse vector
spVector spRopeJointGetImpulse(spRopeJoint* joint);

/// get the joints maximum distance
spFloat spRopeJointGetMaxDistance(spRopeJoint* joint);

/// get the joints impulse magnitude
spFloat spRopeJointGetImpulseLength(spRopeJoint* joint);

/// set body A's anchor point in A's local space
void spRopeJointSetLocalAnchorA(spRopeJoint* joint, spVector anchorA);

/// set body B's anchor point in B's local space
void spRopeJointSetLocalAnchorB(spRopeJoint* joint, spVector anchorB);

/// set body A's anchor point in world space
void spRopeJointSetWorldAnchorA(spRopeJoint* joint, spVector anchorA);

/// set body B's anchor point in world space
void spRopeJointSetWorldAnchorB(spRopeJoint* joint, spVector anchorB);

/// set the max distance between the joints two anchor points
void spRopeJointSetMaxDistance(spRopeJoint* joint, spFloat maxDistance);

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

/// apply the last timesteps impulse, for a more stable simulation
void spRopeJointApplyCachedImpulse(spRopeJoint* joint, const spFloat h);

/// setup a rope joint to be solved
void spRopeJointPreSolve(spRopeJoint* joint, const spFloat h);

/// solve the rope joint, and apply impulses to the bodies
void spRopeJointSolve(spRopeJoint* joint);

#endif