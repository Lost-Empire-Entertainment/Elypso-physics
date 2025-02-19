//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#pragma once

#ifdef _WIN32
	#ifdef PHYSICS_DLL_EXPORT
		#define PHYSICS_API __declspec(dllexport)
	#else
		#define PHYSICS_API __declspec(dllimport)
	#endif
#else
	#define PHYSICS_API
#endif

#include <memory>

//external
#include "glm.hpp"
#include "gtc/quaternion.hpp"

//physics
#include "gameobjecthandle.hpp"
#include "collider.hpp"

namespace ElypsoPhysics
{
	using glm::vec3;
	using glm::quat;
	using std::unique_ptr;

	class PHYSICS_API RigidBody
	{
	public:
		GameObjectHandle handle;       //Reference to the gameobject
		vec3 position;                 //Object position
		quat rotation;                 //Object rotation
		vec3 velocity;                 //Linear velocity
		vec3 angularVelocity;          //Angular velocity
		float mass;                    //Object mass
		bool isDynamic;                //Determines if the object moves
		unique_ptr<Collider> collider; //Smart pointer to the collider
		vec3 inertiaTensor;            //Store precomputed inertia tensor
		bool useGravity;			   //Object gravity state

		RigidBody(
			GameObjectHandle h,
			const vec3& pos,
			const quat& rot,
			float m = 1.0f) :
			handle(h),
			position(pos),
			rotation(rot),
			velocity(0.0f),
			angularVelocity(0.0f),
			mass(m),
			isDynamic(m > 0.0f),
			collider(nullptr) 
		{
			ComputeInertiaTensor();
		}

		/// <summary>
		/// Apply linear force
		/// </summary>
		void ApplyForce(const vec3& force);
		/// <summary>
		/// Apply an instant impulse (collision effects)
		/// </summary>
		void ApplyImpulse(const vec3& impulse);
		/// <summary>
		/// Apply rotational force
		/// </summary>
		void ApplyTorque(const vec3& torque);
		/// <summary>
		/// Precompute inertia tensor
		/// </summary>
		void ComputeInertiaTensor();
	};
}