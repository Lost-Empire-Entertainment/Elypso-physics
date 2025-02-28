//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <iostream>
#include <string>

//physics
#include "rigidbody.hpp"

using std::cout;
using std::string;
using std::to_string;

namespace ElypsoPhysics
{
	RigidBody::RigidBody(
		GameObjectHandle h,
		const vec3& offsetPosition,
		const vec3& combinedPosition,
		const quat& offsetRotation,
		const quat& combinedRotation,
		float m,
		float rest,
		float staticFrict,
		float dynamicFrict,
		float gFactor) :
		handle(h),
		offsetPosition(offsetPosition),
		combinedPosition(combinedPosition),
		offsetRotation(offsetRotation),
		combinedRotation(combinedRotation),
		velocity(0.0f),
		angularVelocity(0.0f),
		mass(m),
		isDynamic(false),
		collider(nullptr),
		restitution(rest),
		staticFriction(staticFrict),
		dynamicFriction(dynamicFrict),
		gravityFactor(gFactor),
		useGravity(false),
		inertiaTensor(vec3(1.0f))
	{
		ComputeInertiaTensor();
	}

	void RigidBody::ApplyForce(const vec3& force)
	{
		//static objects cant move
		if (!isDynamic) return;

		WakeUp();

		vec3 acceleration = force / mass;
		velocity += acceleration;

/*
#ifdef NDEBUG
#else
		uint32_t index = handle.index;
		uint32_t gen = handle.generation;
		string forceValue = to_string(force.x) + ", " + to_string(force.y) + ", " + to_string(force.z);
		string message = "[ELYPSO-PHYSICS | SUCCESS] Applied force '" + forceValue + "' to rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
		cout << message;
#endif
*/
	}

	void RigidBody::ApplyImpulse(const vec3& impulse)
	{
		//static objects cant move
		if (!isDynamic) return;

		WakeUp();

		velocity += impulse / mass;

/*
#ifdef NDEBUG
#else
		uint32_t index = handle.index;
		uint32_t gen = handle.generation;
		string impulseValue = to_string(impulse.x) + ", " + to_string(impulse.y) + ", " + to_string(impulse.z);
		string message = "[ELYPSO-PHYSICS | SUCCESS] Applied impulse '" + impulseValue + "' to rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
		cout << message;
#endif
*/
	}

	void RigidBody::ApplyTorque(const vec3& torque)
	{
		//static objects cant move
		if (!isDynamic) return;

		WakeUp();

		angularVelocity += torque / inertiaTensor;

/*
#ifdef NDEBUG
#else
		uint32_t index = handle.index;
		uint32_t gen = handle.generation;
		string torqueValue = to_string(torque.x) + ", " + to_string(torque.y) + ", " + to_string(torque.z);
		string message = "[ELYPSO-PHYSICS | SUCCESS] Applied torque '" + torqueValue + "' to rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
		cout << message;
#endif
*/
	}

	void RigidBody::ComputeInertiaTensor(const vec3& scale)
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

	void RigidBody::SetCollider(
		const vec3& offsetScale,
		const vec3& combinedScale,
		ColliderType type)
	{
		if (collider) delete collider;

		if (type == ColliderType::BOX)
		{
			collider = new BoxCollider(
				offsetPosition,
				combinedPosition,
				handle);

#ifdef NDEBUG
#else
			uint32_t index = handle.index;
			uint32_t gen = handle.generation;
			string sizeString = to_string(combinedScale.x) + ", " + to_string(combinedScale.y) + ", " + to_string(combinedScale.z);
			string message = "[ELYPSO-PHYSICS | SUCCESS] Set size to '" + sizeString + "' and collider to box for rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
			cout << message;
#endif
		}
		else if (type == ColliderType::SPHERE)
		{
			collider = new SphereCollider(
				offsetPosition,
				combinedPosition,
				handle);

#ifdef NDEBUG
#else
			uint32_t index = handle.index;
			uint32_t gen = handle.generation;
			string radius = to_string(combinedScale.x);
			string message = "[ELYPSO-PHYSICS | SUCCESS] Set radius to '" + radius + "' and collider to sphere for rigidbody (" + to_string(index) + ", " + to_string(gen) + ")!\n";
			cout << message;
#endif
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