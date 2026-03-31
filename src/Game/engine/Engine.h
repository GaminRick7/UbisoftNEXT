#pragma once
#include "ecs/managers/ComponentManager.h"
#include "ecs/managers/EntityManager.h"
#include "ecs/managers/SystemManager.h"
#include "graphics/ModelManager.h"
#include "graphics/CameraSystem.h"
#include "graphics/RenderSystem.h"
#include "graphics/UISystem.h"
#include "scene/SceneManager.h"
#include "spatial/SpatialPartitionManager.h"
#include "spatial/Frustum.h"
#include "spatial/AABB.h"
#include "spatial/Sphere.h"
#include <vector>
#include <cstdint>

// Engine class acts as a facade for all core functions that will be used by gameplay
// Brief overview:
// * ECS management
// * Model loading/management
// * Scene management
// * Event Management

class Engine {
public:
	static void Init(Engine* engine);
	static void Update(float dt);
	static void Render();
	static void SetTimeScale(float scale);
	static float GetTimeScale();

	// ECS Methods
	static Entity CreateEntity();
	static void DestroyEntity(const Entity& e);

	template <typename T>
	static void RegisterComponent() {
		s_instance->componentManager.registerComponent<T>();
	}

	template <typename T>
	static ComponentTypeID GetTypeID() {
		return ComponentTypeRegistry::GetTypeID<T>();
	}

	template <typename T>
	static T& GetComponent(Entity e) {
		assert(s_instance->entityManager.isAlive(e));
		return s_instance->componentManager.getComponent<T>(e.id);
	}

	template <typename T>
	static void AddComponent(const Entity& e, const T& component) {
		assert(s_instance->entityManager.isAlive(e));
		s_instance->componentManager.addComponent<T>(e.id, component);
		ComponentMask mask = s_instance->entityManager.getComponentMask(e.id);
		mask.set(ComponentTypeRegistry::GetTypeID<T>());
		s_instance->entityManager.setComponentMask(e.id, mask);
		s_instance->systemManager.entityMaskChanged(e, mask);
		s_instance->spManager.EntityMaskChanged(e, mask);
	}

	template <typename T>
	static void RemoveComponent(const Entity& e) {
		assert(s_instance->entityManager.isAlive(e));
		s_instance->componentManager.removeComponent<T>(e.id);
		ComponentMask mask = s_instance->entityManager.getComponentMask(e.id);
		mask.reset(ComponentTypeRegistry::GetTypeID<T>());
		s_instance->entityManager.setComponentMask(e.id, mask);
		s_instance->systemManager.entityMaskChanged(e, mask);
		s_instance->spManager.EntityMaskChanged(e, mask);
	}

	template <typename T>
	static bool HasComponent(const Entity& e) {
		if (!s_instance->entityManager.isAlive(e)) {
			return false;
		}
		return s_instance->componentManager.hasComponent<T>(e.id);
	}

	template <typename T>
	static void RegisterSystem() {
		s_instance->systemManager.registerSystem<T>();
	}

	template <typename T>
	static T& GetSystem() {
		return s_instance->systemManager.getSystem<T>();
	}

	static void ClearAllEntities();
	static uint32_t GetEntityCount();
	static void SetActiveCamera(Entity cameraEntity);
	static bool IsActiveCamera(Entity cameraEntity);

	// Model Manager methods
	static std::shared_ptr<Model> loadModel(std::string name);
	static void RegisterScene(std::shared_ptr<Scene> scene);

	// Scene Manager methods
	static std::shared_ptr<Scene> GetScene(const std::string& name);
	static void SwitchScene(const std::string& name);
	static std::shared_ptr<Scene> GetActiveScene();
	static bool SceneExists(const std::string& name);
	static void UpdateActiveScene(float deltaTime);
	static void ClearScenes();

	// Spatial Partitioning Queries
	static std::vector<Entity> QueryFrustum(const Frustum& frustum);
	static std::vector<Entity> QueryAABB(const AABB& bounds);
	static std::vector<Entity> QuerySphere(const Sphere& sphere);
	static std::vector<std::pair<Entity, float>> QueryRay(const Ray& ray);
	static std::vector<AABB> GetOctreeNodeBounds();
	static void ResetSpatialPartition();

private:
	// ECS!
	EntityManager entityManager;
	ComponentManager componentManager;
	SystemManager systemManager;

	// Models
	ModelManager modelManager;

	// Scenes
	SceneManager sceneManager;

	// Spatial Partitioning Manager (/System?)
	// This can be a system since it keeps track of entities and updates them based on their components
	// but, since it feels like a more integral part of the engine and will be used by other systems, the decision is to
	// name it a manager. it's all just semantics :P
	SpatialPartitionManager spManager;

	// for global access and to give the engine's functions a namespace kind of feel!
	// aside: initially a singleton class but static passing makes dependency less hidden
	static inline Engine* s_instance = nullptr;
	static inline float s_timeScale = 1.0f;
};
