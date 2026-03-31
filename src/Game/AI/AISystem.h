#pragma once
#include "engine/ecs/core/System.h"
#include "engine/event/Subject.h"

class AISystem : public System, public Subject {
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();

private:
	bool checkForAgents(Entity& entity);
};
