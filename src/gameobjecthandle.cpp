//Copyright(C) 2025 Lost Empire Entertainment
//This program comes with ABSOLUTELY NO WARRANTY.
//This is free software, and you are welcome to redistribute it under certain conditions.
//Read LICENSE.md for more information.

//physics
#include "gameobjecthandle.hpp"

namespace std
{
	template <>
	struct hash<ElypsoPhysics::GameObjectHandle>
	{
		size_t operator()(const ElypsoPhysics::GameObjectHandle& handle) const
		{
			return hash<uint32_t>()(handle.index) ^ (hash<uint32_t>()(handle.generation) << 1);
		}
	};
}