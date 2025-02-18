//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "physicsworld.hpp"
#include "collisiondetection.hpp"

using std::swap;
using glm::normalize;
using glm::length;

namespace ElypsoPhysics
{
	PhysicsWorld::PhysicsWorld()
	{

	}

	PhysicsWorld::~PhysicsWorld()
	{
		for (RigidBody& body : bodies)
		{
			delete body.collider; //free dynamically allocated colliders
		}
		bodies.clear();
		bodyMap.clear();
		generations.clear();
	}

	GameObjectHandle PhysicsWorld::CreateRigidBody(
		const vec3& pos,
		const quat& rot,
		float mass)
	{
		uint32_t index = static_cast<uint32_t>(bodies.size());
		uint32_t generation = generations.size() > index ? generations[index] : 0;

		GameObjectHandle handle(index, generation);

		bodies.emplace_back(handle, pos, rot, mass);
		bodyMap[handle] = index;

		if (generations.size() <= index) generations.push_back(0);

		return handle;
	}

	RigidBody* PhysicsWorld::GetRigidBody(const GameObjectHandle& handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			return &bodies[it->second];
		}
		return nullptr;
	}

	void PhysicsWorld::RemoveRigidBody(GameObjectHandle handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			size_t index = it->second;

			generations[handle.index]++;
			bodyMap.erase(it);

			if (index < bodies.size() - 1)
			{
				swap(bodies[index], bodies.back());
				bodyMap[bodies[index].handle] = index;
			}

			bodies.pop_back();
		}
	}

	void PhysicsWorld::StepSimulation(float deltaTime)
	{
		//collision detection and resolution
		for (size_t i = 0; i < bodies.size(); i++)
		{
			RigidBody& bodyA = bodies[i];

			for (size_t j = i + 1; j < bodies.size(); j++)
			{
				RigidBody& bodyB = bodies[j];

				if (bodyA.collider
					&& bodyB.collider)
				{
					if (CollisionDetection::CheckAABBCollision(bodyA, bodyB))
					{
						//sphere-sphere collision
						if (bodyA.collider->type == ColliderType::SPHERE
							&& bodyB.collider->type == ColliderType::SPHERE)
						{
							SphereCollider* sphereA = static_cast<SphereCollider*>(bodyA.collider);
							SphereCollider* sphereB = static_cast<SphereCollider*>(bodyB.collider);

							if (CollisionDetection::CheckSphereSphereCollision(
								*sphereA,
								bodyA.position,
								*sphereB,
								bodyB.position))
							{
								//simple push-back collision resolution
								vec3 collisionVector = bodyA.position - bodyB.position;
								if (length(collisionVector) > 0.0f)
								{
									vec3 collisionNormal = normalize(collisionVector);
									bodyA.position += collisionNormal * 0.1f;
									bodyB.position -= collisionNormal * 0.1f;
								}
							}
						}

						//box-box collision
						else if (bodyA.collider->type == ColliderType::BOX
							&& bodyB.collider->type == ColliderType::BOX)
						{
							BoxCollider* boxA = static_cast<BoxCollider*>(bodyA.collider);
							BoxCollider* boxB = static_cast<BoxCollider*>(bodyB.collider);

							if (CollisionDetection::CheckBoxBoxCollision(
								*boxA,
								bodyA.position,
								*boxB,
								bodyB.position))
							{
								//simply push-back collision resolution
								vec3 collisionVector = bodyA.position - bodyB.position;
								if (length(collisionVector) > 0.0f)
								{
									vec3 collisionNormal = normalize(collisionVector);
									bodyA.position += collisionNormal * 0.1f;
									bodyB.position -= collisionNormal * 0.1f;
								}
							}
						}
					}
				}
			}
		}

		//apply physics integration for all bodies
		for (RigidBody& body : bodies)
		{
			if (!body.isDynamic) continue;

			//apply simple euler integration
			body.position += body.velocity * deltaTime;

			//apply angular velocity
			quat angularRotation = quat(
				0,
				body.angularVelocity.x,
				body.angularVelocity.y,
				body.angularVelocity.z)
				* body.rotation
				* 0.5f
				* deltaTime;

			body.rotation += angularRotation;
			body.rotation = normalize(body.rotation);

			//apply basic damping
			body.velocity *= 0.99f;
			body.angularVelocity *= 0.98f;
		}
	}
}