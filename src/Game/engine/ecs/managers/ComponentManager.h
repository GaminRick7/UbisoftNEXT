#pragma once
#include <vector>
#include <cstdint>
#include <cassert>
#include <memory>
#include "../pools/ComponentPool.h"

using ComponentTypeID = uint32_t;

class ComponentTypeRegistry
{
public:
	// each component type gets a unique ID on first call, same ID on calls after that
	template <typename T>
	static ComponentTypeID GetTypeID() {
		static ComponentTypeID typeID = m_nextTypeID++;
		return typeID;
	}
private:
	static inline ComponentTypeID m_nextTypeID = 0;
};

// holds all the pools for different component types!
class ComponentManager {
public:
	template <typename T>
	void registerComponent() {
		ComponentTypeID typeID = ComponentTypeRegistry::GetTypeID<T>();
		
		if (isRegistered<T>()) {
			return;
		}

		if (typeID >= m_pools.size()) {
			m_pools.resize(typeID + 1);
		}

		// each component type gets its own pool
		auto pool = std::make_unique<ComponentPool<T>>();
		m_pools[typeID].reset(pool.release());
	}

	template <typename T>
	void addComponent(EntityID entity, const T& component) {
		assert(isRegistered<T>());
		ComponentTypeID typeID = ComponentTypeRegistry::GetTypeID<T>();
		ComponentPool<T>* pool = static_cast<ComponentPool<T>*>(m_pools[typeID].get());
		pool->add(entity, component);
	}

	template <typename T>
	bool hasComponent(EntityID entity) const {
		if (!isRegistered<T>()) {
			return false;
		}

		ComponentTypeID typeID = ComponentTypeRegistry::GetTypeID<T>();
		const ComponentPool<T>* pool = static_cast<const ComponentPool<T>*>(m_pools[typeID].get());
		return pool->has(entity);
	}

	template <typename T>
	void removeComponent(EntityID entity) {
		assert(isRegistered<T>());
		ComponentTypeID typeID = ComponentTypeRegistry::GetTypeID<T>();
		ComponentPool<T>* pool = static_cast<ComponentPool<T>*>(m_pools[typeID].get());
		pool->remove(entity);
	}

	template <typename T>
	T& getComponent(EntityID entity) {
		assert(isRegistered<T>());
		ComponentTypeID typeID = ComponentTypeRegistry::GetTypeID<T>();
		ComponentPool<T>* pool = static_cast<ComponentPool<T>*>(m_pools[typeID].get());
		return pool->get(entity);
	}

	void clearAll();
	// called when an entity is destroyed to wipe every component it owned
	void removeAllComponents(EntityID entity);

private:
	std::vector<std::unique_ptr<IComponentPool>> m_pools;

	template <typename T>
	bool isRegistered() const {
		ComponentTypeID typeID = ComponentTypeRegistry::GetTypeID<T>();
		return typeID < m_pools.size() && m_pools[typeID] != nullptr;
	}
};
