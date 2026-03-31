#pragma once
#include <vector>
#include "../pools/EntityPool.h"

class SystemManager;
class ComponentManager;
class SpatialPartitionManager;

// Manager for entity lifetimes
class EntityManager {
public:
	void init();
	void update(SystemManager& systemManager, ComponentManager& componentManager, SpatialPartitionManager& spManager);
	void shutdown();

	Entity createEntity();
	// mark for destruction so we can remove it once all systems have run for the frame
	void queueDestroy(const Entity& e);
	// removes any entities that were queued for destruction
	void flushDestroyed(SystemManager& systemManager, ComponentManager& componentManager, SpatialPartitionManager& spManager);

	ComponentMask getComponentMask(const EntityID& id);
	void setComponentMask(const EntityID& e, const ComponentMask& mask);

	bool isAlive(Entity e) const;

	std::vector<Entity> getAllAliveEntities() const;
	uint32_t getLivingCount() const;
	void clearAllEntities();

private:
	EntityPool m_pool;
	// holds entities that should be destroyed once systems finish using them
	std::vector<Entity> m_pendingDestroy{};
};
