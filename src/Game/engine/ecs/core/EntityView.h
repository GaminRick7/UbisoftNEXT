#pragma once
#include <cassert>
#include <vector>
#include <array>
#include <bitset>
#include "Entity.h"
#include "constants.h"

static constexpr size_t INVALID_INDEX = static_cast<size_t>(-1);


// Used by system to easily query components with a certain signature
class EntityView {
public:
	EntityView(const ComponentMask& mask);
	bool matches(const ComponentMask& entityMask);
	bool contains(Entity e) const;
	void add(Entity e);
	void remove(Entity e);
	std::vector<Entity>& dense();

private:
	ComponentMask m_mask;

	// maintains a sparse-dense mapping!
	std::vector<Entity> m_entities;
	std::array<size_t, MAX_ENTITIES> sparse;
};
