//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

#include <iostream>
#include <string>

//external
#include "gtc/quaternion.hpp"

//physics
#include "physicsworld.hpp"
#include "collisiondetection.hpp"

using glm::normalize;
using glm::length;
using std::cerr;
using std::cout;
using std::min;
using std::max;
using glm::clamp;
using std::string;
using std::to_string;
using std::swap;
using glm::acos;
using glm::quat;
using glm::cross;

namespace ElypsoPhysics
{
	PhysicsWorld& PhysicsWorld::GetInstance()
	{
		static PhysicsWorld instance;
		return instance;
	}

	PhysicsWorld::PhysicsWorld() :
		gravity(0.0f, -9.81f, 0.0f),
		isInitialized(false) {
	}
	PhysicsWorld::~PhysicsWorld()
	{
		if (!isInitialized)
		{
			cerr << "[ELYPSO-PHYSICS | ERROR] Cannot shut down Elypso Physics because it has not yet been initialized!\n";
			return;
		}

		for (auto* rb : bodies)
		{
			if (rb)
			{
				GameObjectHandle handle = rb->handle;
				RemoveRigidBody(handle);
			}
		}

		bodies.clear();        //clear the bodies vector
		bodyMap.clear();       //clear the body map
		generations.clear();   //clear the generations map
		isInitialized = false;

#ifdef NDEBUG
#else
		cout << "[ELYPSO-PHYSICS | SUCCESS] Shutdown completed!\n";
#endif
	}

	void PhysicsWorld::InitializePhysics(const vec3& newGravity)
	{
		if (isInitialized)
		{
			cerr << "[ELYPSO-PHYSICS | ERROR] Elypso Physics is already initialized!\n";
			return;
		}

		bodies.clear();
		bodyMap.clear();
		generations.clear();

		gravity = newGravity;

		isInitialized = true;
#ifdef NDEBUG
#else
		cout << "[ELYPSO-PHYSICS | SUCCESS] Initialization completed!\n";
#endif
	}

	GameObjectHandle PhysicsWorld::CreateRigidBody(
		const vec3& offsetPosition,
		const vec3& combinedPosition,
		const quat& offsetRotation,
		const quat& combinedRotation,
		ColliderType colliderType,
		const vec3& colliderSizeOrRadius,
		float mass,
		float restitution,
		float staticFriction,
		float dynamicFriction,
		float gravityFactor,
		bool useGravity)
	{
		if (!isInitialized)
		{
			cerr << "[ELYPSO-PHYSICS | ERROR] Cannot create a RigidBody if Elypso Physics isnt initialized!\n";
			return GameObjectHandle(UINT32_MAX, UINT32_MAX);
		}

		uint32_t index = static_cast<uint32_t>(bodies.size());
		uint32_t generation = generations.size() > index ? generations[index] : 0;

		GameObjectHandle handle(index, generation);

		//create the rigidbody
		RigidBody* rb = new RigidBody(
			handle,
			offsetPosition,
			combinedPosition,
			offsetRotation,
			combinedRotation,
			mass,
			restitution,
			staticFriction,
			dynamicFriction,
			gravityFactor);

		//assign collider based on collider type
		rb->SetCollider(
			vec3(0.0f),
			vec3(1.0f),
			colliderType);

		bodies.push_back(rb);
		bodyMap[handle] = index;

		if (generations.size() <= index) generations.push_back(0);

#ifdef NDEBUG
#else
		string message = "[ELYPSO-PHYSICS | SUCCESS] Created rigidbody (" + to_string(index) + ", " + to_string(generation) + ")!\n";
		cout << message;
#endif

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

	void PhysicsWorld::RemoveRigidBody(const GameObjectHandle& handle)
	{
		auto it = bodyMap.find(handle);
		if (it != bodyMap.end())
		{
			size_t index = it->second;

			if (bodies[index])
			{
				//delete collider first if it exists
				if (bodies[index]->collider)
				{
					delete bodies[index]->collider;
					bodies[index]->collider = nullptr;
				}

				//delete the rigid body
				delete bodies[index];
				bodies[index] = nullptr;
			}

			generations[handle.index]++;
			bodyMap.erase(it);
			if (index < bodies.size() - 1)
			{
				swap(bodies[index], bodies.back());
				bodyMap[bodies[index]->handle] = index;
			}

#ifdef NDEBUG
#else
			uint32_t idx = handle.index;
			uint32_t gen = handle.generation;
			string message = "[ELYPSO-PHYSICS | SUCCESS] Removed rigidbody (" + to_string(idx) + ", " + to_string(gen) + ")!\n";
			cout << message;
#endif

			bodies.pop_back();
		}
	}

	void PhysicsWorld::StepSimulation(float deltaTime)
	{
		if (bodyMap.size() == 0) return;

		for (size_t i = 0; i < bodies.size(); i++)
		{
			RigidBody& bodyA = *bodies[i];

			for (size_t j = i + 1; j < bodies.size(); j++)
			{
				RigidBody& bodyB = *bodies[j];

				if (bodyA.isSleeping
					&& bodyB.isSleeping)
				{
					continue;
				}

				if (!bodyA.collider
					|| !bodyB.collider)
				{
					continue;
				}

				float maxDistance = bodyA.collider->boundingRadius + bodyB.collider->boundingRadius;
				if (length(bodyA.combinedPosition - bodyB.combinedPosition) > maxDistance) continue;

				bool useOBB = false;

				//check if either object has rotation (not identity quaternion)
				if (bodyA.collider->type == ColliderType::BOX
					&& bodyB.collider->type == ColliderType::BOX)
				{
					if (acos(clamp(dot(bodyA.combinedRotation, quat(1, 0, 0, 0)), -1.0f, 1.0f)) > 0.01f
						|| acos(clamp(dot(bodyB.combinedRotation, quat(1, 0, 0, 0)), -1.0f, 1.0f)) > 0.01f)
					{
						useOBB = true;
					}
				}

				
				if (useOBB)
				{
					ContactManifold manifold = CollisionDetection::GenerateOBBContactManifold(bodyA, bodyB);
					if (manifold.colliding
						&& !manifold.contacts.empty())
					{
						for (const auto& contact : manifold.contacts)
						{
							ResolveCollision(bodyA, bodyB, contact.normal, contact.point, contact.penetration);

							ApplyFriction(bodyA, bodyB, contact.normal, contact.point);
						}
					}
				}
			}
		}

		ApplyPhysicsIntegration(deltaTime);
	}

	void PhysicsWorld::ApplyPhysicsIntegration(float deltaTime)
	{
		for (auto& bodyPtr : bodies)
		{
			RigidBody& body = *bodyPtr;

			if (!body.isDynamic) continue;

			if (body.useGravity)
			{
				vec3 gravityImpulse = (gravity * body.gravityFactor) * deltaTime;
				body.ApplyImpulse(gravityImpulse * body.mass);
			}

			vec3 futurePosition = body.combinedPosition + body.velocity * deltaTime;

			//check future collision before applying movement
			for (auto& otherBodyPtr : bodies)
			{
				if (bodyPtr == otherBodyPtr) continue;

				RigidBody& otherBody = *otherBodyPtr;

				if (!otherBody.collider) continue;

				if (CollisionDetection::CheckOBBCollisionAt(body, futurePosition, otherBody))
				{
					body.velocity = vec3(0.0f);
					break;
				}
			}

			//apply simple Euler integration
			body.combinedPosition += body.velocity * deltaTime;

			//apply angular velocity
			if (length(body.angularVelocity) > 0.001f)
			{
				quat angularRotation = quat(
					0,
					body.angularVelocity.x,
					body.angularVelocity.y,
					body.angularVelocity.z)
					* 0.5f
					* deltaTime;

				body.combinedRotation += angularRotation * body.combinedRotation;
				body.combinedRotation = normalize(body.combinedRotation);
			}

			float linearDampingFactor = pow(0.99f, deltaTime * 60.0f);
			float angularDampingFactor = pow(angularDamping, deltaTime * 60.0f);

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
				body.sleepTimer = 0.0f;
				body.WakeUp();
			}
		}
	}

	void PhysicsWorld::ResolveCollision(
		RigidBody& bodyA, 
		RigidBody& bodyB, 
		const vec3& collisionNormal,
		const vec3& contactPoint,
		float penetration)
	{
		//compute relative velocity
		vec3 relativeVelocity = bodyB.velocity - bodyA.velocity;

		//compute velocity along the collision normal
		float velocityAlongNormal = dot(relativeVelocity, collisionNormal);

		//if objects are separating, do nothing
		if (velocityAlongNormal > 0.0f) return;

		//combute combied restitution (take the minimum)
		float restitution = clamp(min(bodyA.restitution, bodyB.restitution), 0.0f, 0.5f);

		//compute contact offsets
		vec3 rA = contactPoint - bodyA.combinedPosition;
		vec3 rB = contactPoint - bodyB.combinedPosition;

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

		vec3 impulse = impulseScalar * collisionNormal;

		bodyA.ApplyImpulse(-impulse);
		bodyB.ApplyImpulse(impulse);

		vec3 torqueImpulseA = cross(rA, impulse);
		vec3 torqueImpulseB = cross(rB, impulse);

		bodyA.ApplyTorque(-torqueImpulseA);
		bodyB.ApplyTorque(torqueImpulseB);

		//clamp small residual angular velocities
		if (length(bodyA.angularVelocity) < 0.01f)
		{
			bodyA.ApplyTorque(-bodyA.angularVelocity * bodyA.inertiaTensor);
		}

		if (length(bodyB.angularVelocity) < 0.01f)
		{
			bodyB.ApplyTorque(-bodyB.angularVelocity * bodyB.inertiaTensor);
		}

		float corrected = max(0.0f, penetration - minPenetrationThreshold);
		if (corrected > 0.0f)
		{
			float totalMass = bodyA.mass + bodyB.mass;
			if (totalMass > 0.00001f)
			{
				float ratioA = (bodyB.mass / totalMass);
				float ratioB = (bodyA.mass / totalMass);

				vec3 correctionVec = collisionNormal * (corrected * correctionFactor);

				bodyA.combinedPosition -= correctionVec * ratioA;
				bodyB.combinedPosition += correctionVec * ratioB;
			}
		}
	}

	void PhysicsWorld::ApplyFriction(
		RigidBody& bodyA, 
		RigidBody& bodyB, 
		const vec3& collisionNormal,
		const vec3& contactPoint) const
	{
		//compute relative velocity at the contact point
		vec3 rA = contactPoint - bodyA.combinedPosition;
		vec3 rB = contactPoint - bodyB.combinedPosition;

		vec3 vA = bodyA.velocity + cross(bodyA.angularVelocity, rA);
		vec3 vB = bodyB.velocity + cross(bodyB.angularVelocity, rB);
		vec3 relativeVelocity = vB - vA;

		//tangential direction
		vec3 tangent = relativeVelocity - dot(relativeVelocity, collisionNormal) * collisionNormal;
		if (length(tangent) < 1e-6f) return;
		tangent = normalize(tangent);

		//compute static and dynamic friction coefficients (use average)
		float staticFriction = (bodyA.staticFriction + bodyB.staticFriction) * frictionMultiplier;
		float dynamicFriction = (bodyA.dynamicFriction + bodyB.dynamicFriction) * frictionMultiplier;

		//compute friction impulse magnitude
		float frictionImpulseScalar = -dot(relativeVelocity, tangent);
		frictionImpulseScalar /= (1.0f / bodyA.mass) + (1.0f / bodyB.mass);

		vec3 frictionImpulse = frictionImpulseScalar * tangent;

		//static friction check
		float maxStaticFriction = staticFriction * length(frictionImpulse);
		if (abs(frictionImpulseScalar) > maxStaticFriction)
		{
			frictionImpulse = dynamicFriction * frictionImpulse;
		}

		bodyA.ApplyImpulse(-frictionImpulse);
		bodyB.ApplyImpulse(frictionImpulse);

		vec3 torqueA = cross(rA, -frictionImpulse);
		vec3 torqueB = cross(rB, frictionImpulse);

		bodyA.ApplyTorque(torqueA);
		bodyB.ApplyTorque(torqueB);

		if (length(bodyA.velocity) < 0.01f)
		{
			bodyA.angularVelocity *= lowAngularVelocityFactor;
		}

		if (length(bodyB.velocity) < 0.01f)
		{
			bodyB.angularVelocity *= lowAngularVelocityFactor;
		}
	}
}