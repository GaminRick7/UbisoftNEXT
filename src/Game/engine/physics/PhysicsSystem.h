#pragma once
#include "engine/ecs/core/System.h"
#include "engine/event/Observer.h"
#include "math/vec3.h"

class PhysicsSystem : public System, public Observer {
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();
private:
	const Vec3 gravity = Vec3(0.0f, -9.81f, 0.0f);
	const float sleepThresholdSq = 1e-6f;

	void OnNotify(EventType type, const Event& payload) override;
};
