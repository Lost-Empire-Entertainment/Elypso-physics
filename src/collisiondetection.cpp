//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <cmath>

//physics
#include "collisiondetection.hpp"

using glm::dot;
using std::abs;

namespace ElypsoPhysics
{
	bool CollisionDetection::CheckAABBCollision(const RigidBody& a, const RigidBody& b)
	{
		//no collision if no colliders
		if (!a.collider
			|| !b.collider)
		{
			return false;
		}

		vec3 extentsA;
		if (a.collider->type == ColliderType::BOX)
		{
			extentsA = static_cast<BoxCollider*>(a.collider.get())->halfExtents;
		}
		else if (a.collider->type == ColliderType::SPHERE)
		{
			float radius = static_cast<SphereCollider*>(a.collider.get())->radius;
			extentsA = vec3(radius);
		}
		else extentsA = vec3(0);

		vec3 extentsB;
		if (b.collider->type == ColliderType::BOX)
		{
			extentsB = static_cast<BoxCollider*>(b.collider.get())->halfExtents;
		}
		else if (b.collider->type == ColliderType::SPHERE)
		{
			float radius = static_cast<SphereCollider*>(b.collider.get())->radius;
			extentsB = vec3(radius);
		}
		else extentsB = vec3(0);

		//compute AABB min/max for both objects
		vec3 minA = a.position - extentsA;
		vec3 maxA = a.position + extentsA;
		vec3 minB = b.position - extentsB;
		vec3 maxB = b.position + extentsB;

		return (minA.x <= maxB.x && maxA.x >= minB.x) 
			&& (minA.y <= maxB.y && maxA.y >= minB.y) 
			&& (minA.z <= maxB.z && maxA.z >= minB.z);
	}

	bool CollisionDetection::CheckBoxBoxCollision(
		const BoxCollider& boxA,
		const vec3& posA,
		const BoxCollider& boxB,
		const vec3& posB)
	{
		return (abs(posA.x - posB.x) <= (boxA.halfExtents.x + boxB.halfExtents.x)
			&& abs(posA.y - posB.y) <= (boxA.halfExtents.y + boxB.halfExtents.y)
			&& abs(posA.z - posB.z) <= (boxA.halfExtents.z + boxB.halfExtents.z));
	}

	bool CollisionDetection::CheckSphereSphereCollision(
		const SphereCollider& sphereA,
		const vec3& posA,
		const SphereCollider& sphereB,
		const vec3& posB)
	{
		float radiusSum = sphereA.radius + sphereB.radius;
		float distanceSquared = dot(posA - posB, posA - posB);
		return distanceSquared <= (radiusSum * radiusSum);
	}
}