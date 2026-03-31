#pragma once
#include "engine/ecs/core/System.h"
#include "engine/event/Observer.h"
#include <random>

class ArrowSystem : public System, public Observer {
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();
private:
	void OnNotify(EventType type, const Event& payload) override;
	float m_timeSeconds = 0.0f;
	float m_floatAmplitude = 0.4f;
	float m_floatSpeed = 2.0f;
	std::mt19937 m_rng;
	std::uniform_real_distribution<float> m_phaseDist{0.0f, 1.0f};
};
