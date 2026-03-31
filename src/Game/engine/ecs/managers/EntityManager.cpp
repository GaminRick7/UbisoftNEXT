#include "EntityManager.h"
#include "SystemManager.h"
#include "ComponentManager.h"
#include "../../spatial/SpatialPartitionManager.h"

void EntityManager::init() {
	m_pool.init();
}

void EntityManager::update(SystemManager& systemManager,
	ComponentManager& componentManager, SpatialPartitionManager& spManager) {
	// remove the entities that were queued for destruction thios frame
	flushDestroyed(systemManager, componentManager, spManager);
}

void EntityManager::shutdown() {

}

Entity EntityManager::createEntity() {
	return m_pool.createEntity();
}

void EntityManager::queueDestroy(const Entity& e) {
	m_pendingDestroy.emplace_back(e);
}

void EntityManager::flushDestroyed(SystemManager& systemManager, ComponentManager& componentManager, SpatialPartitionManager& spManager) {
	for (const auto& entity : m_pendingDestroy) {
		ComponentMask mask = getComponentMask(entity.id);
		
		// notify systems that entity is being destroyed
		systemManager.entityDestroyed(entity, mask);
		
		// notify spatial partition manager
		spManager.EntityDestroyed(entity, mask);
		
		// remove all components from the entity
		componentManager.removeAllComponents(entity.id);
		
		// finally, destroy the entity in the pool
		m_pool.destroyEntity(entity);
	}
	m_pendingDestroy.clear();
}

ComponentMask EntityManager::getComponentMask(const EntityID& id) {
	return m_pool.getComponentMask(id);
}

void EntityManager::setComponentMask(const EntityID& e, const ComponentMask& mask) {
	m_pool.setComponentMask(e, mask);
}

bool EntityManager::isAlive(Entity e) const {
	return m_pool.isAlive(e);
}

std::vector<Entity> EntityManager::getAllAliveEntities() const {
	return m_pool.getAllAliveEntities();
}

uint32_t EntityManager::getLivingCount() const {
	return m_pool.getLivingCount();
}

void EntityManager::clearAllEntities() {
	m_pendingDestroy.clear();
	m_pool.clearAll();
}




