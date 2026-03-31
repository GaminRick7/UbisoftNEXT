#pragma once
#include "Entity.h"
#include "EntityView.h"


// Base System Class
class System {
public:
	virtual ~System() = default;
	// views are automatically managed by this base class on entity changes
	// so actual systems do not have to deal with maintaining its views
	void addEntityView(const ComponentMask& mask);
	void ClearEntityViews();
	void EntityMaskChanged(Entity e, ComponentMask mask);
	void EntityDestroyed(Entity e, ComponentMask mask);
protected:
	std::vector<EntityView> m_entityViews;
};
