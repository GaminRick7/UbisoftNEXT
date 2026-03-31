#pragma once
#include "engine/ecs/core/System.h"

class TimeControlSystem : public System{
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();
};
