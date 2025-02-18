//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "rigidbody.hpp"

namespace ElypsoPhysics
{
	void RigidBody::ApplyForce(const vec3& force)
	{
		//static objects cant move
		if (!isDynamic) return;

		vec3 acceleration = force / mass;
		velocity += acceleration;
	}

	void RigidBody::ApplyTorque(const vec3& torque)
	{
		//static objects cant move
		if (!isDynamic) return;

		//simple angular velocity update, assuming mass represets moment of inertia
		angularVelocity += torque / mass;
	}
}