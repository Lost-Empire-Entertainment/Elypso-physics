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
using std::min;
using glm::clamp;

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
		RigidBody* rb = new RigidBody(
			handle,
			pos,
			rot,
			mass,
			restitution,
			staticFriction,
			dynamicFriction,
			gravityFactor);

		//assign collider based on collider type
		if (colliderType == ColliderType::BOX)
		{
			rb->collider = new BoxCollider(handle, colliderSizeOrRadius);
		}
		else if (colliderType == ColliderType::SPHERE)
		{
			rb->collider = new SphereCollider(handle, colliderSizeOrRadius.x);
		}

		bodies.push_back(rb);
		bodyMap[handle] = index;

		if (generations.size() <= index) generations.push_back(0);

		return handle;
	}

	RigidBody* PhysicsWorld::GetRigidBody(const GameObjectHandle& handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			return bodies[it->second];
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
				bodyMap[bodies[index]->handle] = index;
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
				float maxDistance = bodyA.collider->boundingRadius + bodyB.collider->boundingRadius;
				if (length(bodyA.position - bodyB.position) > maxDistance) continue;

				if (CollisionDetection::CheckAABBCollision(bodyA, bodyB))
				{
					vec3 collisionVector = bodyA.position - bodyB.position;
					if (length(collisionVector) > 0.0f)
					{
						vec3 collisionNormal = normalize(collisionVector);
						vec3 delta = bodyA.position - bodyB.position;
						vec3 overlap = abs(delta);

						if (overlap.x < overlap.y 
							&& overlap.x < overlap.z)
						{
							collisionNormal = vec3((delta.x > 0) ? 1.0f : -1.0f, 0.0f, 0.0f);
						}
						else if (overlap.y < overlap.z)
						{
							collisionNormal = vec3(0.0f, (delta.y > 0) ? 1.0f : -1.0f, 0.0f);
						}
						else
						{
							collisionNormal = vec3(0.0f, 0.0f, (delta.z > 0) ? 1.0f : -1.0f);
						}

						bodyA.WakeUp();
						bodyB.WakeUp();

						//compute penetration depth based on actual overlap
						float penetrationDepth = maxDistance - length(bodyA.position - bodyB.position);

						vec3 contactPoint =
							bodyB.position + collisionNormal
							* (bodyB.collider->boundingRadius
							- penetrationDepth * 0.5f);

						//apply impulse-based collision response
						ResolveCollision(bodyA, bodyB, collisionNormal, contactPoint);

						if (penetrationDepth > 0.0f)
						{
							float totalMass = bodyA.mass + bodyB.mass;
							float ratioA = bodyB.mass / totalMass;
							float ratioB = bodyA.mass / totalMass;

							const float correctionFactor = 0.8f;
							const float maxCorrection = 0.5f;

							vec3 correction = collisionNormal * min(penetrationDepth * correctionFactor, maxCorrection);
							bodyA.position += correction * ratioA;
							bodyB.position -= correction * ratioB;
						}

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

			vec3 futurePosition = body.position + body.velocity * deltaTime;

			//check future collision before applying movement
			for (auto& otherBodyPtr : bodies)
			{
				if (bodyPtr == otherBodyPtr) continue;

				RigidBody& otherBody = *otherBodyPtr;

				if (!otherBody.collider) continue;

				if (CollisionDetection::CheckAABBCollisionAt(body, futurePosition, otherBody))
				{
					body.velocity = vec3(0.0f);
					break;
				}
			}

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

			float linearDampingFactor = pow(0.99f, deltaTime * 60.0f);
			float angularDampingFactor = pow(0.98f, deltaTime * 60.0f);

			body.velocity *= linearDampingFactor;
			body.angularVelocity *= angularDampingFactor;

			if (length(body.velocity) < body.sleepThreshold
				&& length(body.angularVelocity) < body.sleepThreshold)
			{
				body.sleepTimer += deltaTime;
				if (body.sleepTimer > 2.0f)
				{
					body.Sleep();
				}
			}
			else
			{
				body.sleepTimer = 0.0f; //reset if there's movement
				body.WakeUp();
			}
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

		if (denominator < 0.0001f) return;

		float impulseScalar = -(1.0f + restitution) * velocityAlongNormal / denominator;

		//clamp the impulse to prevent excessive velocity spikes
		const float maxImpulse = 50.0f;
		impulseScalar = clamp(impulseScalar, -maxImpulse, maxImpulse);

		//apply linear impulse
		vec3 impulse = impulseScalar * collisionNormal;
		bodyA.velocity -= impulse * invMassA;
		bodyB.velocity += impulse * invMassB;

		//apply angular impulse (torque)
		vec3 torqueImpulseA = cross(rA, impulse) * invInertiaA;
		vec3 torqueImpulseB = cross(rB, impulse) * invInertiaB;

		bodyA.angularVelocity -= torqueImpulseA;
		bodyB.angularVelocity += torqueImpulseB;

		//clamp small residual angular velocities
		if (length(bodyA.angularVelocity) < 0.01f) bodyA.angularVelocity = vec3(0.0f);
		if (length(bodyB.angularVelocity) < 0.01f) bodyB.angularVelocity = vec3(0.0f);
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
		if (abs(frictionImpulseScalar) < maxStaticFriction)
		{
			//apply full static friction
			bodyA.velocity -= frictionImpulse / bodyA.mass;
			bodyB.velocity += frictionImpulse / bodyB.mass;
		}
		else
		{
			//apply dynamic friction instead
			frictionImpulse = dynamicFriction * frictionImpulse;
			bodyA.velocity -= frictionImpulse / bodyA.mass;
			bodyB.velocity += frictionImpulse / bodyB.mass;
		}

		if (length(bodyA.velocity) < 0.01f) bodyA.velocity = vec3(0.0f);
		if (length(bodyB.velocity) < 0.01f) bodyB.velocity = vec3(0.0f);
	}
}