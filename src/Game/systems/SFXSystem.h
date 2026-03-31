#pragma once
#include "engine/ecs/core/System.h"
#include "engine/event/Observer.h"

class SFXSystem : public System, public Observer {
public:
	void Init();
	void Shutdown();
private:
	void OnNotify(EventType type, const Event& payload) override;
};
