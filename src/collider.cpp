//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "collider.hpp"

namespace ElypsoPhysics
{
	BoxCollider::BoxCollider(
		GameObjectHandle h,
		const vec3& size)
		: Collider(ColliderType::BOX, h),
		halfExtents(size * 0.5f) {}

	SphereCollider::SphereCollider(
		GameObjectHandle h,
		float r)
		: Collider(ColliderType::SPHERE, h),
		radius(r) {}
}