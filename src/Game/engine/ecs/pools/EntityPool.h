#pragma once
#include <bitset>
#include <array>
#include <vector>
#include "../core/Entity.h"
#include "../core/constants.h"

// tracks generation info for each slot
struct EntityRecord {
	Generation generation = 0;
	bool alive = false;
};

// gives out entity ids + keeps their masks
class EntityPool {
public:
	void init();

	Entity createEntity();
	void destroyEntity(const Entity& e);

	ComponentMask getComponentMask(const EntityID& id);
	void setComponentMask(const EntityID& e, const ComponentMask& mask);

	bool isAlive(Entity e) const;

	uint32_t getLivingCount() const { return m_livingCount; }

	void clearAll();
	
	std::vector<Entity> getAllAliveEntities() const;

private:
	std::array<EntityRecord, MAX_ENTITIES> m_records;
	std::array<ComponentMask, MAX_ENTITIES> m_componentMasks;

	// stack that stores the available entities
	// initially, this was a queue but a stack's LIFO structure 
	// would make it much better for reusing cached entities c:
	std::array<EntityID, MAX_ENTITIES> m_freeStack{};
	uint32_t m_freeTop = 0;

	uint32_t m_livingCount = 0;
};
