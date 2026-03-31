#include "EntityView.h"

EntityView::EntityView(const ComponentMask& mask) : m_mask(mask) {
	sparse.fill(INVALID_INDEX);
}

bool EntityView::matches(const ComponentMask& entityMask) {
	return (m_mask & entityMask) == m_mask;
}

bool EntityView::contains(Entity e) const {
	size_t index = sparse[e.id];
	if (index == INVALID_INDEX) return false;

	return m_entities[index].generation == e.generation;
}

void EntityView::add(Entity e) {
	if (contains(e)) return;

	sparse[e.id] = m_entities.size();
	m_entities.push_back(e);
}

void EntityView::remove(Entity e) {
	if (!contains(e)) return;

	size_t index = sparse[e.id];
	assert(index != INVALID_INDEX);

	Entity last = m_entities.back();

	m_entities[index] = last;
	sparse[last.id] = index;

	m_entities.pop_back();
	sparse[e.id] = INVALID_INDEX;
}

std::vector<Entity>& EntityView::dense() {
	return m_entities;
}