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

#include <string>
#include <iostream>

//external
#include "glm.hpp"

//physics
#include "gameobjecthandle.hpp"

namespace ElypsoPhysics
{
	using glm::vec3;
	using std::string;
	using std::to_string;
	using std::cout;

	enum class ColliderType
	{
		BOX,
		SPHERE
	};

	class PHYSICS_API Collider
	{
	public:
		ColliderType type;
		GameObjectHandle handle;
		vec3 localScale;
		vec3 worldScale;
		float boundingRadius = 0.0f;

		explicit Collider(
			const vec3& localScale,
			const vec3& worldScale,
			ColliderType type, 
			const GameObjectHandle& h);
		
		virtual ~Collider() = default;

		virtual void CalculateBoundingRadius() = 0;
		virtual void UpdateScale(const vec3& newWorldScale) = 0;

		Collider(const Collider&) = delete;
		Collider& operator=(const Collider&) = delete;
	};

	class PHYSICS_API BoxCollider : public Collider
	{
	public:
		BoxCollider(
			const vec3& localScale,
			const vec3& worldScale,
			const GameObjectHandle& h);
		void UpdateScale(const vec3& newWorldScale) override
		{
			halfExtents = newWorldScale * 0.5f;
			CalculateBoundingRadius();
		}
		void CalculateBoundingRadius() override
		{
			boundingRadius = length(halfExtents) * 2.0f * 0.5f; //half diagonal
		}
		//half size of box in each axis
		vec3 halfExtents;
	};

	class PHYSICS_API SphereCollider : public Collider
	{
	public:
		SphereCollider(
			const vec3& localScale,
			const vec3& worldScale,
			const GameObjectHandle& h);
		void UpdateScale(const vec3& newWorldScale) override
		{
			radius = newWorldScale.x * 0.5f;
			CalculateBoundingRadius();
		}
		void CalculateBoundingRadius() override
		{
			boundingRadius = radius;
		}

		float radius;
	};
}