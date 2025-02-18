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

#include <vector>
#include <unordered_map>

//external
#include "glm.hpp"

//physics
#include "rigidbody.hpp"

namespace ElypsoPhysics
{
	using std::vector;
	using std::unordered_map;
	using glm::vec3;
	using glm::quat;

	class PHYSICS_API PhysicsWorld
	{
	public:
		PhysicsWorld();

		/// <summary>
		/// Create a RigidBody and return its handle.
		/// </summary>
		GameObjectHandle CreateRigidBody(
			const vec3& pos, 
			const quat& rot,
			float mass = 1.0f);

		/// <summary>
		/// Get a RigidBody by handle.
		/// </summary>
		RigidBody* GetRigidBody(GameObjectHandle handle);

		/// <summary>
		/// Remove a RigidBody
		/// </summary>
		void RemoveRigidBody(GameObjectHandle handle);

		/// <summary>
		/// Update physics simulation
		/// </summary>
		void StepSimulation(float deltaTime);

	private:
		vector<RigidBody> bodies;                        //Array of all rigidbodies
		unordered_map<GameObjectHandle, size_t> bodyMap; //Map for quick lookup
		vector<uint32_t> generations;                    //Tracks generation of each index
	};
}