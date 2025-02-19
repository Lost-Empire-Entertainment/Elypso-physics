//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <iostream>

//physics
#include "physicsworld.hpp"
#include "collisiondetection.hpp"

using std::swap;
using glm::normalize;
using glm::length;
using std::cerr;
using std::cout;
using std::make_unique;

namespace ElypsoPhysics
{
	PhysicsWorld& PhysicsWorld::GetInstance()
	{
		static PhysicsWorld instance;
		return instance;
	}

	PhysicsWorld::PhysicsWorld() {}

	PhysicsWorld::~PhysicsWorld()
	{
		if (isInitialized)
		{
			cerr << "Error: Elypso Physics was not properly shut down. Call ShutdownPhysics() before destruction.\n";
		}
	}

	void PhysicsWorld::InitializePhysics(const vec3& newGravity)
	{
		if (isInitialized)
		{
			cerr << "Error: Elypso Physics is already initialized!\n";
			return;
		}

		bodies.clear();
		bodyMap.clear();
		generations.clear();

		gravity = newGravity;

		isInitialized = true;

		cout << "Initialized Elypso Physics!\n";
	}

	void PhysicsWorld::ShutdownPhysics()
	{
		if (!isInitialized)
		{
			cerr << "Error: Cannot shut down Elypso Physics because it has not yet been initialized!\n";
			return;
		}

		bodies.clear();
		bodyMap.clear();
		generations.clear();
		isInitialized = false;

		cout << "Successfully shut down Elypso Physics!\n";
	}

	GameObjectHandle PhysicsWorld::CreateRigidBody(
		const vec3& pos,
		const quat& rot,
		float mass)
	{
		uint32_t index = static_cast<uint32_t>(bodies.size());
		uint32_t generation = generations.size() > index ? generations[index] : 0;

		GameObjectHandle handle(index, generation);

		bodies.emplace_back(make_unique<RigidBody>(handle, pos, rot, mass));
		bodyMap[handle] = index;

		if (generations.size() <= index) generations.push_back(0);

		return handle;
	}

	RigidBody* PhysicsWorld::GetRigidBody(const GameObjectHandle& handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			return bodies[it->second].get();
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
				bodyMap[bodies[index].get()->handle] = index;
			}

			bodies.erase(bodies.begin() + index);
		}
	}

	void PhysicsWorld::StepSimulation(float deltaTime)
	{
		//collision detection and resolution
		for (size_t i = 0; i < bodies.size(); i++)
		{
			RigidBody& bodyA = *bodies[i];

			for (size_t j = i + 1; j < bodies.size(); j++)
			{
				RigidBody& bodyB = *bodies[j];

				if (bodyA.collider.get()
					&& bodyB.collider.get())
				{
					if (CollisionDetection::CheckAABBCollision(bodyA, bodyB))
					{
						//sphere-sphere collision
						if (bodyA.collider.get()->type == ColliderType::SPHERE
							&& bodyB.collider.get()->type == ColliderType::SPHERE)
						{
							SphereCollider* sphereA = static_cast<SphereCollider*>(bodyA.collider.get());
							SphereCollider* sphereB = static_cast<SphereCollider*>(bodyB.collider.get());

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

									//calculate mass-based displacement ratios
									float totalMass = bodyA.mass + bodyB.mass;
									float ratioA = bodyB.mass / totalMass;
									float ratioB = bodyA.mass / totalMass;

									//apply proportional movement based on mass
									bodyA.position += collisionNormal * (0.1f * ratioA);
									bodyB.position -= collisionNormal * (0.1f * ratioB);
								}
							}
						}

						//box-box collision
						else if (bodyA.collider.get()->type == ColliderType::BOX
							&& bodyB.collider.get()->type == ColliderType::BOX)
						{
							BoxCollider* boxA = static_cast<BoxCollider*>(bodyA.collider.get());
							BoxCollider* boxB = static_cast<BoxCollider*>(bodyB.collider.get());

							if (CollisionDetection::CheckBoxBoxCollision(
								*boxA,
								bodyA.position,
								*boxB,
								bodyB.position))
							{
								vec3 collisionVector = bodyA.position - bodyB.position;
								if (length(collisionVector) > 0.0f)
								{
									vec3 collisionNormal = normalize(collisionVector);

									//calculate mass-based displacement ratios
									float totalMass = bodyA.mass + bodyB.mass;
									float ratioA = bodyB.mass / totalMass;
									float ratioB = bodyA.mass / totalMass;

									//apply proportional movement based on mass
									bodyA.position += collisionNormal * (0.1f * ratioA);
									bodyB.position -= collisionNormal * (0.1f * ratioB);
								}
							}
						}
					}
				}
			}
		}

		//apply physics integration for all bodies
		for (auto& bodyPtr : bodies)
		{
			RigidBody& body = *bodyPtr;

			if (!body.isDynamic) continue;

			body.velocity += gravity * deltaTime;

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