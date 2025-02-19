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
						vec3 collisionVector = bodyA.position - bodyB.position;
						if (length(collisionVector) > 0.0f)
						{
							vec3 collisionNormal = normalize(collisionVector);

							//apply impulse-based collision response
							ResolveCollision(bodyA, bodyB, collisionNormal);

							//apply friction
							ApplyFriction(bodyA, bodyB, collisionNormal);
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

			if (body.useGravity) body.velocity += gravity * deltaTime;

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

	void PhysicsWorld::ResolveCollision(RigidBody& bodyA, RigidBody& bodyB, const vec3& collisionNormal)
	{
		//compute relative velocity
		vec3 relativeVelocity = bodyB.velocity - bodyA.velocity;

		//compute velocity along the collision normal
		float velocityAlongNormal = dot(relativeVelocity, collisionNormal);

		//if objects are separating, do nothing
		if (velocityAlongNormal > 0.0f) return;

		//get restitution (bounciness)
		float restitution = 0.5f; //should be changed to per-object

		//compute impulse scalar
		float impulseScalar = -(1.0f + restitution) * velocityAlongNormal;
		impulseScalar /= (1.0f / bodyA.mass) + (1.0f / bodyB.mass);

		//apply impulse
		vec3 impulse = impulseScalar * collisionNormal;
		bodyA.velocity -= impulse / bodyA.mass;
		bodyB.velocity += impulse / bodyB.mass;
	}

	void PhysicsWorld::ApplyFriction(RigidBody& bodyA, RigidBody& bodyB, const vec3& collisionNormal)
	{
		//compute relative velocity
		vec3 relativeVelocity = bodyB.velocity - bodyA.velocity;

		//compute tangetial velocity (velocity perpendicular to the normal)
		vec3 tangent = relativeVelocity - dot(relativeVelocity, collisionNormal) * collisionNormal;

		//no significant tangential velocity
		if (length(tangent) < 0.001f) return;

		tangent = normalize(tangent);

		//friction coefficient
		float friction = 0.3f; //should be changed to per-object

		//compute friction impulse
		float frictionImpulseScalar = -dot(relativeVelocity, tangent);
		frictionImpulseScalar /= (1.0f / bodyA.mass) + (1.0f / bodyB.mass);
		frictionImpulseScalar *= friction;

		vec3 frictionImpulse = frictionImpulseScalar * tangent;

		//apply friction impulse
		bodyA.velocity -= frictionImpulse / bodyA.mass;
		bodyB.velocity += frictionImpulse / bodyB.mass;
	}
}