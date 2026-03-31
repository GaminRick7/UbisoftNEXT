#include "ComponentManager.h"

void ComponentManager::clearAll() {
	for (auto& pool : m_pools) {
		if (pool) {
			pool->clear();
		}
	}
}

void ComponentManager::removeAllComponents(EntityID entity) {
	for (auto& pool : m_pools) {
		if (pool) {
			pool->EntityDestroyed(entity);
		}
	}
}
