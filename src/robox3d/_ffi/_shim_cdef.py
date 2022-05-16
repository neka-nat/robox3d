"""cffi definitions for the robox3d batch shim (csrc/shim.h).

The shim API is small and stable, so it is maintained by hand (unlike box3d
itself, it is not auto-generated). Keep it in sync with csrc/shim.h.
"""

SHIM_CDEF = r"""
void rxBodyGetPoses( const b3BodyId* ids, int count, float* out );
void rxBodyGetVelocities( const b3BodyId* ids, int count, float* out );
void rxRevoluteGetAngles( const b3JointId* ids, int count, float* out );
void rxRevoluteGetSpeeds( const b3JointId* ids, int count, float* out );
void rxRevoluteSetTargets( const b3JointId* ids, int count, const float* targets );
void rxRevoluteSetMotorSpeeds( const b3JointId* ids, int count, const float* speeds );
void rxRevoluteSetMaxMotorTorques( const b3JointId* ids, int count, const float* torques );
void rxJointGetConstraintLoads( const b3JointId* ids, int count, float* out );
void rxWorldStepN( b3WorldId worldId, float timeStep, int subStepCount, int n );
void rxCastRaysClosest( b3WorldId worldId, const float* origins, const float* directions,
                        int count, float* outFractions, uint8_t* outHits );
"""
