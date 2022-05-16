// robox3d batch shim: bulk API for cutting the number of FFI calls in hot loops.
// Rather than mirroring box3d's API directly, it provides only array-wise get/set.
#pragma once

#include "box3d/box3d.h"

#if defined( _WIN32 )
#define RX_API __declspec( dllexport )
#else
#define RX_API __attribute__( ( visibility( "default" ) ) )
#endif

// Bulk-read body poses. out has count*7 elements (x, y, z, qx, qy, qz, qw)
RX_API void rxBodyGetPoses( const b3BodyId* ids, int count, float* out );

// Bulk-read body velocities. out has count*6 elements (vx, vy, vz, wx, wy, wz)
RX_API void rxBodyGetVelocities( const b3BodyId* ids, int count, float* out );

// Bulk-read revolute joint angles. out has count elements (rad)
RX_API void rxRevoluteGetAngles( const b3JointId* ids, int count, float* out );

// Bulk-read revolute joint angular velocities. out has count elements (rad/s)
// Relative angular velocity about the hinge axis (Z axis of joint frame A).
RX_API void rxRevoluteGetSpeeds( const b3JointId* ids, int count, float* out );

// Bulk-set spring target angles. targets has count elements (rad)
RX_API void rxRevoluteSetTargets( const b3JointId* ids, int count, const float* targets );

// Bulk-set motor speeds. speeds has count elements (rad/s)
RX_API void rxRevoluteSetMotorSpeeds( const b3JointId* ids, int count, const float* speeds );

// Bulk-set motor max torques. torques has count elements (N·m). For pseudo torque control
RX_API void rxRevoluteSetMaxMotorTorques( const b3JointId* ids, int count, const float* torques );

// Bulk-read joint constraint forces and torques. out has count*6 elements (fx, fy, fz, tx, ty, tz)
RX_API void rxJointGetConstraintLoads( const b3JointId* ids, int count, float* out );

// Step n times in one call (for RL frame skipping)
RX_API void rxWorldStepN( b3WorldId worldId, float timeStep, int subStepCount, int n );

// Bulk closest-hit raycast (for LiDAR).
// origins/directions have count*3 elements. directions is a unit vector × ray length.
// outFractions has count elements (1.0 when no hit), outHits has count elements (0/1).
RX_API void rxCastRaysClosest( b3WorldId worldId, const float* origins, const float* directions,
							   int count, float* outFractions, uint8_t* outHits );
