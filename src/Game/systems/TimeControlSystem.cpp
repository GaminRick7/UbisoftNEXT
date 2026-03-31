#include "TimeControlSystem.h"

#include "app.h"
#include "engine/Engine.h"

void TimeControlSystem::Init() {
	Engine::SetTimeScale(1.0f);
}

void TimeControlSystem::Update(float deltaTime) {
	if (App::GetController().GetRightTrigger() > 0.2) {
		Engine::SetTimeScale(2.0f);
	}
	else if (App::GetController().GetLeftTrigger() > 0.2) {
		Engine::SetTimeScale(0.5f);
	} else {
		Engine::SetTimeScale(1.0f);
	}
}

void TimeControlSystem::Shutdown() {
	Engine::SetTimeScale(1.0f);
}

