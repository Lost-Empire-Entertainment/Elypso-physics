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

	class PHYSICS_API RigidBody
	{
	public:
		GameObjectHandle handle; //Reference to the gameobject
		vec3 position;           //Object position
		quat rotation;           //Object rotation
		vec3 velocity;           //Linear velocity
		vec3 angularVelocity;    //Angular velocity
		float mass;              //Object mass
		bool isDynamic;          //Determines if the object moves
		Collider* collider;      //Pointer to a collider

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
			collider(nullptr) {}

		void ApplyForce(const vec3& force);
		void ApplyTorque(const vec3& torque);
	};
}