// SPDX-License-Identifier: MIT
#include "shim.h"

void rxBodyGetPoses( const b3BodyId* ids, int count, float* out )
{
	for ( int i = 0; i < count; ++i )
	{
		b3WorldTransform xf = b3Body_GetTransform( ids[i] );
		float* p = out + 7 * i;
		p[0] = (float)xf.p.x;
		p[1] = (float)xf.p.y;
		p[2] = (float)xf.p.z;
		p[3] = xf.q.v.x;
		p[4] = xf.q.v.y;
		p[5] = xf.q.v.z;
		p[6] = xf.q.s;
	}
}

void rxBodyGetVelocities( const b3BodyId* ids, int count, float* out )
{
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 v = b3Body_GetLinearVelocity( ids[i] );
		b3Vec3 w = b3Body_GetAngularVelocity( ids[i] );
		float* p = out + 6 * i;
		p[0] = v.x;
		p[1] = v.y;
		p[2] = v.z;
		p[3] = w.x;
		p[4] = w.y;
		p[5] = w.z;
	}
}

void rxRevoluteGetAngles( const b3JointId* ids, int count, float* out )
{
	for ( int i = 0; i < count; ++i )
	{
		out[i] = b3RevoluteJoint_GetAngle( ids[i] );
	}
}

void rxRevoluteGetSpeeds( const b3JointId* ids, int count, float* out )
{
	for ( int i = 0; i < count; ++i )
	{
		b3BodyId bodyA = b3Joint_GetBodyA( ids[i] );
		b3BodyId bodyB = b3Joint_GetBodyB( ids[i] );
		b3Transform frameA = b3Joint_GetLocalFrameA( ids[i] );
		b3WorldTransform xfA = b3Body_GetTransform( bodyA );
		b3Quat qJoint = b3MulQuat( xfA.q, frameA.q );
		b3Vec3 axis = b3RotateVector( qJoint, b3Vec3_axisZ );
		b3Vec3 wRel = b3Sub( b3Body_GetAngularVelocity( bodyB ), b3Body_GetAngularVelocity( bodyA ) );
		out[i] = b3Dot( wRel, axis );
	}
}

void rxRevoluteSetTargets( const b3JointId* ids, int count, const float* targets )
{
	for ( int i = 0; i < count; ++i )
	{
		b3RevoluteJoint_SetTargetAngle( ids[i], targets[i] );
	}
}

void rxRevoluteSetMotorSpeeds( const b3JointId* ids, int count, const float* speeds )
{
	for ( int i = 0; i < count; ++i )
	{
		b3RevoluteJoint_SetMotorSpeed( ids[i], speeds[i] );
	}
}

void rxRevoluteSetMaxMotorTorques( const b3JointId* ids, int count, const float* torques )
{
	for ( int i = 0; i < count; ++i )
	{
		b3RevoluteJoint_SetMaxMotorTorque( ids[i], torques[i] );
	}
}

void rxJointGetConstraintLoads( const b3JointId* ids, int count, float* out )
{
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 f = b3Joint_GetConstraintForce( ids[i] );
		b3Vec3 t = b3Joint_GetConstraintTorque( ids[i] );
		float* p = out + 6 * i;
		p[0] = f.x;
		p[1] = f.y;
		p[2] = f.z;
		p[3] = t.x;
		p[4] = t.y;
		p[5] = t.z;
	}
}

void rxWorldStepN( b3WorldId worldId, float timeStep, int subStepCount, int n )
{
	for ( int i = 0; i < n; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}
}

void rxCastRaysClosest( b3WorldId worldId, const float* origins, const float* directions, int count,
						float* outFractions, uint8_t* outHits )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	for ( int i = 0; i < count; ++i )
	{
		b3Pos origin = { origins[3 * i], origins[3 * i + 1], origins[3 * i + 2] };
		b3Vec3 translation = { directions[3 * i], directions[3 * i + 1], directions[3 * i + 2] };
		b3RayResult result = b3World_CastRayClosest( worldId, origin, translation, filter );
		outFractions[i] = result.hit ? result.fraction : 1.0f;
		outHits[i] = result.hit ? 1 : 0;
	}
}
