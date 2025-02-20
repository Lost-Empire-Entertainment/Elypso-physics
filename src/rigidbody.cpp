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

		WakeUp();

		vec3 acceleration = force / mass;
		velocity += acceleration;
	}

	void RigidBody::ApplyImpulse(const vec3& impulse)
	{
		//static objects cant move
		if (!isDynamic) return;

		WakeUp();

		velocity += impulse / mass;
	}

	void RigidBody::ApplyTorque(const vec3& torque)
	{
		//static objects cant move
		if (!isDynamic) return;

		WakeUp();

		angularVelocity += torque / inertiaTensor;
	}

	void RigidBody::ComputeInertiaTensor()
	{
		if (!collider) return;

		if (collider->type == ColliderType::BOX)
		{
			BoxCollider* box = static_cast<BoxCollider*>(collider);
			vec3 halfExtents = box->halfExtents;

			float I_x = (1.0f / 12.0f) * mass * (halfExtents.y * halfExtents.y + halfExtents.z * halfExtents.z);
			float I_y = (1.0f / 12.0f) * mass * (halfExtents.x * halfExtents.x + halfExtents.z * halfExtents.z);
			float I_z = (1.0f / 12.0f) * mass * (halfExtents.x * halfExtents.x + halfExtents.y * halfExtents.y);

			inertiaTensor = vec3(I_x, I_y, I_z);
		}
		else if (collider->type == ColliderType::SPHERE)
		{
			SphereCollider* sphere = static_cast<SphereCollider*>(collider);
			float inertia = (2.0f / 5.0f) * mass * (sphere->radius * sphere->radius);
			inertiaTensor = vec3(inertia);
		}
	}

	void RigidBody::WakeUp()
	{
		isSleeping = false;
		sleepTimer = 0.0f;
	}

	void RigidBody::Sleep()
	{
		isSleeping = true;
		velocity = vec3(0.0f);
		angularVelocity = vec3(0.0f);
	}
}