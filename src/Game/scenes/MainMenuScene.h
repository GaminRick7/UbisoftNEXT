#pragma once
#include "engine/ecs/core/Entity.h"
#include "engine/scene/Scene.h"
#include "math/vec3.h"
#include <random>

class MainMenuScene : public Scene {
public:
	MainMenuScene() : Scene("MainMenu") {}

	void Init() override;
	void Update(float deltaTime) override;
	void Shutdown() override;
private:
	Entity m_orbitCamera;
	Entity m_topDownCamera;
	float m_cameraSwitchTimer = 0.0f;
	bool m_useOrbitCamera = true;
	Entity m_promptTextEntity;
	float m_promptFlashTimer = 0.0f;
	float m_promptFlashInterval = 0.5f;
	bool m_promptVisible = true;
	float m_spawnInterval = 0.05f;
	float m_spawnAccumulator = 0.0f;
	int m_spawnedCount = 0;
	int m_agentsPerGroup = 40;
	int m_currentGroupId = -1;
	float m_flowCellSize = 1.0f;
	Vec3 m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);
	std::mt19937 m_rng;
	std::uniform_real_distribution<float> m_spawnJitter{-0.2f, 0.2f};
	std::uniform_real_distribution<float> m_colorDist{0.3f, 1.0f};
	std::uniform_real_distribution<float> m_heightDist{0.3f, 0.6f};
	std::uniform_real_distribution<float> m_widthDist{0.15f, 0.3f};
};
