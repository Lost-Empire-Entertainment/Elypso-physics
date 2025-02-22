//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "collider.hpp"

namespace ElypsoPhysics
{
	Collider::Collider(
		const vec3& localScale,
		const vec3& worldScale,
		ColliderType type, 
		const GameObjectHandle& h) : 
		localScale(localScale),
		worldScale(worldScale),
		type(type), 
		handle(h) {}

	BoxCollider::BoxCollider(
		const vec3& localScale,
		const vec3& worldScale,
		const GameObjectHandle& h) : 
		Collider(
			localScale,
			worldScale,
			ColliderType::BOX, 
			h),
		halfExtents(worldScale * 0.5f)
	{
		CalculateBoundingRadius();
	}

	SphereCollider::SphereCollider(
		const vec3& localScale,
		const vec3& worldScale,
		const GameObjectHandle& h) : 
		Collider(
			localScale,
			worldScale,
			ColliderType::SPHERE, 
			h),
		radius(worldScale.x)
	{
		CalculateBoundingRadius();
	}
}