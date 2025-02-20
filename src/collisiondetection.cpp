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

		//broad-phase radius check
		float maxDistance = a.collider->boundingRadius + b.collider->boundingRadius;
		if (length(a.position - b.position) > maxDistance) return false;

		vec3 extentsA;
		if (a.collider->type == ColliderType::BOX)
		{
			extentsA = static_cast<BoxCollider*>(a.collider)->halfExtents;
		}
		else if (a.collider->type == ColliderType::SPHERE)
		{
			float radius = static_cast<SphereCollider*>(a.collider)->radius;
			extentsA = vec3(radius);
		}
		else extentsA = vec3(0);

		vec3 extentsB;
		if (b.collider->type == ColliderType::BOX)
		{
			extentsB = static_cast<BoxCollider*>(b.collider)->halfExtents;
		}
		else if (b.collider->type == ColliderType::SPHERE)
		{
			float radius = static_cast<SphereCollider*>(b.collider)->radius;
			extentsB = vec3(radius);
		}
		else extentsB = vec3(0);

		//compute AABB min/max for both objects
		vec3 minA = a.position - extentsA;
		vec3 maxA = a.position + extentsA;
		vec3 minB = b.position - extentsB;
		vec3 maxB = b.position + extentsB;

		//add edge tolerance to prevent slipping through edges
		float edgeTolerance = 0.01f;

		return (minA.x <= maxB.x + edgeTolerance && maxA.x >= minB.x - edgeTolerance) 
			&& (minA.y <= maxB.y + edgeTolerance && maxA.y >= minB.y - edgeTolerance) 
			&& (minA.z <= maxB.z + edgeTolerance && maxA.z >= minB.z - edgeTolerance);
	}

	bool CollisionDetection::CheckAABBCollisionAt(const RigidBody& movingBody, const vec3& futurePosition, const RigidBody& otherBody)
	{
		if (!otherBody.collider) return false;

		//calculate future AABB for moving body
		vec3 movingExtents = (movingBody.collider->type == ColliderType::BOX)
			? static_cast<BoxCollider*>(movingBody.collider)->halfExtents
			: vec3(static_cast<SphereCollider*>(movingBody.collider)->radius);

		vec3 futureMinA = futurePosition - movingExtents;
		vec3 futureMaxA = futurePosition + movingExtents;

		//current AABB for other body
		vec3 otherExtents = (otherBody.collider->type == ColliderType::BOX)
			? static_cast<BoxCollider*>(otherBody.collider)->halfExtents
			: vec3(static_cast<SphereCollider*>(otherBody.collider)->radius);

		vec3 minB = otherBody.position - otherExtents;
		vec3 maxB = otherBody.position + otherExtents;

		float edgeTolerance = 0.01f;

		//check future overlap with edge tolerance
		return (futureMinA.x <= maxB.x + edgeTolerance && futureMaxA.x >= minB.x - edgeTolerance)
			&& (futureMinA.y <= maxB.y + edgeTolerance && futureMaxA.y >= minB.y - edgeTolerance)
			&& (futureMinA.z <= maxB.z + edgeTolerance && futureMaxA.z >= minB.z - edgeTolerance);
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