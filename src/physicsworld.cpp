//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "physicsworld.hpp"

using std::swap;

namespace ElypsoPhysics
{
	PhysicsWorld::PhysicsWorld()
	{

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

	RigidBody* PhysicsWorld::GetRigidBody(GameObjectHandle handle)
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
		for (RigidBody& body : bodies)
		{
			if (!body.isDynamic) return;

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