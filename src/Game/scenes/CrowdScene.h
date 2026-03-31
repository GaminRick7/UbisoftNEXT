#pragma once
#include "engine/ecs/core/Entity.h"
#include "engine/scene/Scene.h"
#include "math/vec3.h"
#include "engine/event/Observer.h"
#include <random>

class CrowdScene : public Scene, public Observer {
public:
	CrowdScene() : Scene("CrowdScene") {}

	void Init() override;
	void Update(float deltaTime) override;
	void Shutdown() override;

private:
	void OnNotify(EventType type, const Event& payload) override;

	Entity m_camera;
	Entity m_goalOrbitCamera;
	Vec3 m_goalCenter = Vec3(0.0f, 0.0f, 0.0f);
	Entity m_goalTextEntity;
	Entity m_returnTextEntity;
	Entity m_drone1Entity;
	Entity m_drone1TextEntity;
	bool m_goalTextShown = false;
	bool m_waitingForReturn = false;
	int m_gridX = 40;
	int m_gridZ = 80;
	float m_spacing = 1.5f;
	bool m_flybyActive = true;
	float m_flybyTimer = 0.0f;
	float m_flybyDuration = 6.0f;
	bool m_flybyForward = true;
	Vec3 m_flybyStart;
	Vec3 m_flybyEnd;
	Vec3 m_flybyStartLookAt;
	Vec3 m_flybyEndLookAt;
	float m_spawnInterval = 0.05f;
	float m_spawnAccumulator = 0.0f;
	int m_spawnedCount = 0;
	int m_agentsPerGroup = 50;
	int m_currentGroupId = -1;
	std::mt19937 m_rng;
	std::uniform_real_distribution<float> m_spawnJitter{-0.2f, 0.2f};
	std::uniform_real_distribution<float> m_colorDist{0.3f, 1.0f};
	std::uniform_real_distribution<float> m_heightDist{0.3f, 0.6f};
	std::uniform_real_distribution<float> m_widthDist{0.15f, 0.3f};
};

// TODO:
// rendering codedrone path toggle
// crowd scene
// you wins
// ystem rest
// back to main menu