#include "UISystem.h"

#include "engine/Engine.h"
#include "engine/components/UIPrintComponent.h"
#include "../../../ContestAPI/app.h"

void UISystem::Init() {
	ComponentMask uiMask;
	uiMask.set(Engine::GetTypeID<UIPrintComponent>());
	addEntityView(uiMask);
}

void UISystem::Render() {
	if (m_entityViews.empty()) {
		return;
	}

	auto& uiEntities = m_entityViews[0].dense();
	for (const Entity& entity : uiEntities) {
		const UIPrintComponent& ui = Engine::GetComponent<UIPrintComponent>(entity);
		if (ui.text.empty()) {
			continue;
		}

		if (ui.font) {
			App::Print(ui.x, ui.y, ui.text.c_str(), ui.r, ui.g, ui.b, ui.font);
		} else {
			App::Print(ui.x, ui.y, ui.text.c_str(), ui.r, ui.g, ui.b);
		}
	}
}

void UISystem::Shutdown() {
}
