#include "Engine.h"
#include <vector>

#include "components/MeshComponent.h"
#include "components/LightComponent.h"
#include "components/BoundsComponent.h"
#include "components/RigidbodyComponent.h"
#include "physics/CollisionSystem.h"
#include "physics/PhysicsSystem.h"
#include "graphics/UISystem.h"
#include "components/UIPrintComponent.h"

void Engine::Init(Engine* engine) {
	s_instance = engine;
	s_instance->entityManager.init();

	s_instance->RegisterComponent<CameraComponent>();
	s_instance->RegisterComponent<TransformComponent>();
	s_instance->RegisterComponent<MeshComponent>();
	s_instance->RegisterComponent<LightComponent>();
	s_instance->RegisterComponent<BoundsComponent>();
	s_instance->RegisterComponent<RigidbodyComponent>();
	s_instance->RegisterComponent<ColliderComponent>();
	s_instance->RegisterComponent<UIPrintComponent>();

	s_instance->RegisterSystem<RenderSystem>();
	s_instance->RegisterSystem<UISystem>();
	s_instance->RegisterSystem<CameraSystem>();
	s_instance->RegisterSystem<CollisionSystem>();
	s_instance->RegisterSystem<PhysicsSystem>();

	s_instance->GetSystem<CameraSystem>().Init();
	s_instance->GetSystem<RenderSystem>().Init();
	s_instance->GetSystem<UISystem>().Init();
	s_instance->GetSystem<CollisionSystem>().Init();
	s_instance->GetSystem<PhysicsSystem>().Init();

	s_instance->spManager.Init();
}


void Engine::Update(float dt) {
	const float scaledDt = dt * s_timeScale;
	s_instance->entityManager.update(s_instance->systemManager,
		s_instance->componentManager, s_instance->spManager);
	s_instance->GetSystem<PhysicsSystem>().Update(scaledDt);
	s_instance->spManager.Update(scaledDt);
	s_instance->GetSystem<CollisionSystem>().Update(scaledDt);
	s_instance->GetSystem<CameraSystem>().Update(dt);
	s_instance->GetSystem<RenderSystem>().Update(scaledDt);
}

void Engine::Render() {
	s_instance->GetSystem<RenderSystem>().Render();
	s_instance->GetSystem<UISystem>().Render();
}

Entity Engine::CreateEntity() {
	const Entity& e = s_instance->entityManager.createEntity();
	s_instance->systemManager.entityMaskChanged(e, ComponentMask());

	return e;
}

void Engine::DestroyEntity(const Entity& e) {
	assert(s_instance->entityManager.isAlive(e));
	s_instance->entityManager.queueDestroy(e);
}

void Engine::ClearAllEntities() {
	// get all alive entities before clearing
	std::vector<Entity> aliveEntities = s_instance->entityManager.getAllAliveEntities();
	
	// notify all systems that each entity is being destroyed
	for (const Entity& entity : aliveEntities) {
		ComponentMask mask = s_instance->entityManager.getComponentMask(entity.id);
		s_instance->systemManager.entityDestroyed(entity, mask);
		s_instance->spManager.EntityDestroyed(entity, mask);
	}
	
	// clear component pools
	s_instance->componentManager.clearAll();
	
	// clear entity pool
	s_instance->entityManager.clearAllEntities();
}

uint32_t Engine::GetEntityCount() {
	return s_instance->entityManager.getLivingCount();
}

void Engine::ResetSpatialPartition() {
	s_instance->spManager.Shutdown();
	s_instance->spManager.Init();
}

std::shared_ptr<Model> Engine::loadModel(std::string name) {
	return s_instance->modelManager.loadModel(name);
}

void Engine::RegisterScene(std::shared_ptr<Scene> scene) {
	s_instance->sceneManager.RegisterScene(scene);
}

std::shared_ptr<Scene> Engine::GetScene(const std::string& name) {
	return s_instance->sceneManager.GetScene(name);
}

void Engine::SwitchScene(const std::string& name) {
	s_instance->sceneManager.SwitchScene(name);
}

std::shared_ptr<Scene> Engine::GetActiveScene() {
	return s_instance->sceneManager.GetActiveScene();
}

bool Engine::SceneExists(const std::string& name) {
	return s_instance->sceneManager.SceneExists(name);
}

void Engine::UpdateActiveScene(float deltaTime) {
	s_instance->sceneManager.UpdateActiveScene(deltaTime * s_timeScale);
}

void Engine::SetTimeScale(float scale) {
	s_timeScale = scale;
}

float Engine::GetTimeScale() {
	return s_timeScale;
}

void Engine::ClearScenes() {
	s_instance->sceneManager.ClearScenes();
}

void Engine::SetActiveCamera(Entity cameraEntity) {
	s_instance->GetSystem<RenderSystem>().SetActiveCamera(cameraEntity);
}

bool Engine::IsActiveCamera(Entity cameraEntity) {
	return s_instance->GetSystem<RenderSystem>().IsActiveCamera(cameraEntity);
}

std::vector<Entity> Engine::QueryFrustum(const Frustum& frustum) {
	return s_instance->spManager.QueryFrustum(frustum);
}

std::vector<Entity> Engine::QueryAABB(const AABB& bounds) {
	return s_instance->spManager.QueryAABB(bounds);
}

std::vector<Entity> Engine::QuerySphere(const Sphere& sphere) {
	return s_instance->spManager.QuerySphere(sphere);
}

std::vector<std::pair<Entity, float>> Engine::QueryRay(const Ray& ray) {
	return s_instance->spManager.QueryRay(ray);
}

std::vector<AABB> Engine::GetOctreeNodeBounds() {
	return s_instance->spManager.GetNodeBounds();
}