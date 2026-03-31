#pragma once
#include <cassert>
#include <vector>
#include <typeindex>
#include <memory>
#include <unordered_map>

#include "../pools/EntityPool.h"
#include "../core/System.h"

// manages the concrete systems
class SystemManager {
public:
	template <typename T>
	void registerSystem() {
		std::type_index key(typeid(T));
		
		if (m_systems.find(key) != m_systems.end()) {
			return;
		}
		
		// create on first request so unused systems are never built
		auto system = std::make_unique<T>();
		m_systems[key] = std::move(system);
	}

	template <typename T>
	T& getSystem() {
		// asserts so callers catch missing registrations during development
		std::type_index key(typeid(T));
		assert(m_systems.find(key) != m_systems.end());
		return *static_cast<T*>(m_systems[key].get());
	}

	void entityDestroyed(Entity entity, ComponentMask entityMask) {
		// remove entity from every system
		for (auto& [_, system] : m_systems) {
			system->EntityDestroyed(entity, entityMask);
		}
	}

	void entityMaskChanged(Entity entity, ComponentMask entityMask) {
		// systems decide whether to track or drop the entity based on new mask
		for (auto& [_, system] : m_systems) {
			system->EntityMaskChanged(entity, entityMask);
		}
	}


private:
	std::unordered_map<std::type_index, std::unique_ptr<System>> m_systems;
};
