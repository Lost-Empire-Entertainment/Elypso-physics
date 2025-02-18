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
using std::min;
using std::move;

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
		ColliderType colliderType,
		const vec3& colliderSizeOrRadius,
		float mass,
		float restitution,
		float staticFriction,
		float dynamicFriction,
		float gravityFactor,
		bool useGravity)
	{
		uint32_t index = static_cast<uint32_t>(bodies.size());
		uint32_t generation = generations.size() > index ? generations[index] : 0;

		GameObjectHandle handle(index, generation);

		//create the rigidbody
		auto rb = make_unique<RigidBody>(
			handle,
			pos,
			rot,
			mass,
			restitution,
			staticFriction,
			dynamicFriction,
			gravityFactor,
			useGravity);

		//assign collider based on collider type
		if (colliderType == ColliderType::BOX)
		{
			rb->collider = make_unique<BoxCollider>(handle, colliderSizeOrRadius);
		}
		else if (colliderType == ColliderType::SPHERE)
		{
			rb->collider = make_unique<SphereCollider>(handle, colliderSizeOrRadius.x);
		}

		bodies.push_back(move(rb));
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

				//skip if both bodies are sleeping (no need to check for collision)
				if (bodyA.isSleeping
					&& bodyB.isSleeping)
				{
					continue;
				}

				//skip if one or both bodies are missing collider
				if (!bodyA.collider
					|| !bodyB.collider)
				{
					continue;
				}

				//skip if objects are too far apart
				if (length(bodyA.position - bodyB.position) > 10.0f) continue;

				if (CollisionDetection::CheckAABBCollision(bodyA, bodyB))
				{
					vec3 collisionVector = bodyA.position - bodyB.position;
					if (length(collisionVector) > 0.0f)
					{
						vec3 collisionNormal = normalize(collisionVector);

						bodyA.WakeUp();
						bodyB.WakeUp();

						//compute mass-based displacement
						if (bodyA.mass > 0.0f
							&& bodyB.mass > 0.0f)
						{
							float totalMass = bodyA.mass + bodyB.mass;
							float ratioA = bodyB.mass / totalMass;
							float ratioB = bodyA.mass / totalMass;

							bodyA.position += collisionNormal * (0.1f * ratioA);
							bodyB.position -= collisionNormal * (0.1f * ratioB);
						}

						vec3 contactPoint = (bodyA.position + bodyB.position) * 0.5f;

						//apply impulse-based collision response
						ResolveCollision(bodyA, bodyB, collisionNormal, contactPoint);

						//apply friction
						ApplyFriction(bodyA, bodyB, collisionNormal);
					}
				}
			}
		}

		//apply physics integration for all bodies
		for (auto& bodyPtr : bodies)
		{
			RigidBody& body = *bodyPtr;

			if (!body.isDynamic) continue;

			if (body.useGravity) body.velocity += (gravity * body.gravityFactor) * deltaTime;

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

			if (length(body.velocity) < body.sleepThreshold
				&& length(body.angularVelocity) < body.sleepThreshold)
			{
				body.sleepTimer += deltaTime;
				if (body.sleepTimer > 2.0f) body.Sleep();
			}
			else body.WakeUp();
		}
	}

	void PhysicsWorld::ResolveCollision(
		RigidBody& bodyA, 
		RigidBody& bodyB, 
		const vec3& collisionNormal,
		const vec3& contactPoint)
	{
		//compute relative velocity
		vec3 relativeVelocity = bodyB.velocity - bodyA.velocity;

		//compute velocity along the collision normal
		float velocityAlongNormal = dot(relativeVelocity, collisionNormal);

		//if objects are separating, do nothing
		if (velocityAlongNormal > 0.0f) return;

		//combute combied restitution (take the minimum)
		float restitution = min(bodyA.restitution, bodyB.restitution);

		//compute contact offsets
		vec3 rA = contactPoint - bodyA.position;
		vec3 rB = contactPoint - bodyB.position;

		//compute inverse mass and inverse inertia tensors
		float invMassA = (bodyA.mass > 0.0f) ? (1.0f / bodyA.mass) : 0.0f;
		float invMassB = (bodyB.mass > 0.0f) ? (1.0f / bodyB.mass) : 0.0f;

		vec3 invInertiaA = (bodyA.mass > 0.0f) ? (1.0f / bodyA.inertiaTensor) : vec3(0.0f);
		vec3 invInertiaB = (bodyB.mass > 0.0f) ? (1.0f / bodyB.inertiaTensor) : vec3(0.0f);

		//compute impulse scalar
		vec3 crossRA_N = cross(rA, collisionNormal);
		vec3 crossRB_N = cross(rB, collisionNormal);

		float denominator = invMassA + invMassB
			+ dot(crossRA_N * invInertiaA, crossRA_N)
			+ dot(crossRB_N * invInertiaB, crossRB_N);

		float impulseScalar = -(1.0f + restitution) * velocityAlongNormal / denominator;

		//apply linear impulse
		vec3 impulse = impulseScalar * collisionNormal;
		bodyA.velocity -= impulse * invMassA;
		bodyB.velocity += impulse * invMassB;

		//apply angular impulse (torque)
		vec3 torqueImpulseA = cross(rA, impulse) * invInertiaA;
		vec3 torqueImpulseB = cross(rB, impulse) * invInertiaB;

		bodyA.angularVelocity -= torqueImpulseA;
		bodyB.angularVelocity += torqueImpulseB;
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

		//compute static and dynamic friction coefficients (use average)
		float staticFriction = (bodyA.staticFriction + bodyB.staticFriction) * 0.5f;
		float dynamicFriction = (bodyA.dynamicFriction + bodyB.dynamicFriction) * 0.5f;

		//compute friction impulse magnitude
		float frictionImpulseScalar = -dot(relativeVelocity, tangent);
		frictionImpulseScalar /= (1.0f / bodyA.mass) + (1.0f / bodyB.mass);

		vec3 frictionImpulse = frictionImpulseScalar * tangent;

		//static friction check
		float maxStaticFriction = staticFriction * length(frictionImpulse);
		if (length(frictionImpulse) < maxStaticFriction)
		{
			//apply full static friction
			bodyA.velocity -= frictionImpulse / bodyA.mass;
			bodyB.velocity += frictionImpulse / bodyB.mass;
		}
		else
		{
			//apply dynamic friction instead
			frictionImpulse *= dynamicFriction;
			bodyA.velocity -= frictionImpulse / bodyA.mass;
			bodyB.velocity += frictionImpulse / bodyB.mass;
		}
	}
}