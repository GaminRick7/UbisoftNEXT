
#include "System.h"

void System::addEntityView(const ComponentMask& mask) {
	m_entityViews.emplace_back(mask);
}

void System::ClearEntityViews() {
	m_entityViews.clear();
}

void System::EntityMaskChanged(Entity e, ComponentMask mask) {
	for (auto& view : m_entityViews) {
		if (view.matches(mask)) {
			view.add(e);
		}
		else {
			view.remove(e);
		}
	}
}

void System::EntityDestroyed(Entity e, ComponentMask mask) {
	for (auto& view : m_entityViews) {
		view.remove(e);
	}
}