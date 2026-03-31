#pragma once
#include "engine/ecs/core/System.h"

class UISystem : public System {
public:
	void Init();
	void Render();
	void Shutdown();
};
