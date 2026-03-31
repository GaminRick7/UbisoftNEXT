#include "EntityPool.h"
#include <cassert>

void EntityPool::init() {
	// fill the free stack so we pop fresh ids first
	for (EntityID i = 0; i < MAX_ENTITIES; i++) {
		m_freeStack[i] = MAX_ENTITIES - 1 - i;
	}
	m_freeTop = MAX_ENTITIES;
}


Entity EntityPool::createEntity() {
	assert(m_freeTop > 0 && "MAX_ENTITIES exceeded");

	EntityID id = m_freeStack[--m_freeTop];
	EntityRecord& record = m_records[id];
	record.alive = true;

	m_componentMasks[id].reset();

	++m_livingCount;

	return Entity{ id, record.generation };
}

void EntityPool::destroyEntity(const Entity& e) {
	assert(isAlive(e));

	EntityRecord& record = m_records[e.id];

	//update the generation id to invalidate the previous entity use after destruction
	record.alive = false;
	record.generation++;

	m_freeStack[m_freeTop++] = e.id;
	--m_livingCount;
}


bool EntityPool::isAlive(Entity e) const {
	if (e.id >= MAX_ENTITIES)
		return false;

	const EntityRecord& record = m_records[e.id];
	return record.alive && record.generation == e.generation;
}

ComponentMask EntityPool::getComponentMask(const EntityID& id) {
	assert(id < MAX_ENTITIES);
	return m_componentMasks[id];
}


void EntityPool::setComponentMask(const EntityID& id, const ComponentMask& mask) {
	assert(id < MAX_ENTITIES);
	m_componentMasks[id] = mask;
}

std::vector<Entity> EntityPool::getAllAliveEntities() const {
	std::vector<Entity> aliveEntities;
	aliveEntities.reserve(m_livingCount);
	
	for (EntityID id = 0; id < MAX_ENTITIES; ++id) {
		const EntityRecord& record = m_records[id];
		if (record.alive) {
			// keep the current generation so handles stay valid
			aliveEntities.emplace_back(Entity{ id, record.generation });
		}
	}
	
	return aliveEntities;
}

void EntityPool::clearAll() {
	// reset everything so next scene loads clean
	for (EntityID i = 0; i < MAX_ENTITIES; i++) {
		m_records[i].alive = false;
		m_records[i].generation = 0;
		m_componentMasks[i].reset();
		m_freeStack[i] = MAX_ENTITIES - 1 - i;
	}
	m_freeTop = MAX_ENTITIES;
	m_livingCount = 0;
}
