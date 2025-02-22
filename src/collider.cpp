//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "collider.hpp"

namespace ElypsoPhysics
{
	Collider::Collider(
		const vec3& offsetScale,
		const vec3& combinedScale,
		ColliderType type, 
		const GameObjectHandle& h) : 
		offsetScale(offsetScale),
		combinedScale(combinedScale),
		type(type), 
		handle(h) {}

	BoxCollider::BoxCollider(
		const vec3& offsetScale,
		const vec3& combinedScale,
		const GameObjectHandle& h) : 
		Collider(
			offsetScale,
			combinedScale,
			ColliderType::BOX, 
			h),
		halfExtents(combinedScale * 0.5f)
	{
		CalculateBoundingRadius();
	}

	SphereCollider::SphereCollider(
		const vec3& offsetScale,
		const vec3& combinedScale,
		const GameObjectHandle& h) : 
		Collider(
			offsetScale,
			combinedScale,
			ColliderType::SPHERE, 
			h),
		radius(combinedScale.x)
	{
		CalculateBoundingRadius();
	}
}