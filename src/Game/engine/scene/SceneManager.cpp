#include "SceneManager.h"
#include "../Engine.h"
#include "../graphics/CameraSystem.h"
#include "../graphics/RenderSystem.h"
#include "../graphics/UISystem.h"
#include "../physics/CollisionSystem.h"
#include "../physics/PhysicsSystem.h"
#include "../spatial/SpatialPartitionManager.h"
#include "../../AI/AISystem.h"
#include "../../AI/NavSystem.h"
#include "../../systems/ArrowSystem.h"
#include "../../systems/CrowdSimulationSystem.h"
#include "../../systems/FlowFieldSystem.h"
#include "../../systems/SFXSystem.h"
#include "../../systems/TimeControlSystem.h"

void SceneManager::RegisterScene(std::shared_ptr<Scene> scene) {
	if (!scene) {
		return;
	}

	std::string name = scene->GetName();
	if (SceneExists(name)) {
		return;
	}

	m_scenes[name] = scene;
}

std::shared_ptr<Scene> SceneManager::GetScene(const std::string& name) {
	auto it = m_scenes.find(name);
	if (it != m_scenes.end()) {
		return it->second;
	}
	return nullptr;
}

void SceneManager::SwitchScene(const std::string& name) {
	auto scene = GetScene(name);
	if (!scene) {
		return;
	}

	// Shutdown current scene if any
	if (m_activeScene) {
		m_activeScene->Shutdown();
	}

	// Clear all entities from previous scene
	Engine::ClearAllEntities();
	Engine::ResetSpatialPartition();
	ResetSystems();

	// Initialize and activate new scene
	m_activeScene = scene;
	scene->Init();
}

void SceneManager::ResetSystems() {

	// so that we can reset systems before switching to new scene
	auto reset = [](auto& system) {
		system.Shutdown();
		system.ClearEntityViews();
		system.Init();
	};

	reset(Engine::GetSystem<RenderSystem>());
	reset(Engine::GetSystem<CameraSystem>());
	reset(Engine::GetSystem<UISystem>());
	reset(Engine::GetSystem<CollisionSystem>());
	reset(Engine::GetSystem<PhysicsSystem>());
	reset(Engine::GetSystem<FlowFieldSystem>());
	reset(Engine::GetSystem<ArrowSystem>());
	reset(Engine::GetSystem<CrowdSimulationSystem>());
	reset(Engine::GetSystem<AISystem>());
	reset(Engine::GetSystem<NavSystem>());
	reset(Engine::GetSystem<TimeControlSystem>());
	reset(Engine::GetSystem<SFXSystem>());
}

void SceneManager::UpdateActiveScene(float deltaTime) {
	if (m_activeScene) {
		Engine::Update(deltaTime);
		m_activeScene->Update(deltaTime);
	}
}

void SceneManager::ClearScenes() {
	if (m_activeScene) {
		m_activeScene->Shutdown();
		m_activeScene = nullptr;
	}

	Engine::ClearAllEntities();
	Engine::ResetSpatialPartition();
	m_scenes.clear();
}

bool SceneManager::SceneExists(const std::string& name) const {
	return m_scenes.find(name) != m_scenes.end();
}

