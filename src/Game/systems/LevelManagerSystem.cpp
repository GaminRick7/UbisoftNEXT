#include "LevelManagerSystem.h"

#include <random>

#include "engine/Engine.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/ColliderComponent.h"
#include "engine/components/LightComponent.h"
#include "engine/components/MeshComponent.h"
#include "engine/components/RigidbodyComponent.h"

void LevelManagerSystem::Init() {
	// ComponentMask mask;
	// addEntityView()

}

void LevelManagerSystem::CreateLevel() {
	std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<float> colorDist(0.2f, 1.0f);
	std::uniform_real_distribution<float> heightDist(1.0, 20.0);

	for (int i = 0; i < m_grid_w; i++) {
		for (int j = 0; j < m_grid_d; j++) {
			Entity e = Engine::CreateEntity();
			TransformComponent t;
			float y_height = heightDist(rng);
			t.position = Vec3(20 * i, y_height, 20 * j);
			t.scale = Vec3(10, y_height, 10);
			Engine::AddComponent(e, t);

			MeshComponent m;
			m.mesh = Engine::loadModel("./data/TestData/cube.obj");
			m.colour = Vec3(colorDist(rng), colorDist(rng), colorDist(rng));
			Engine::AddComponent(e, m);

			BoundsComponent bounds_component;
			bounds_component.bounds = m.mesh->bounds;
			Engine::AddComponent(e, bounds_component);

			Entity light = Engine::CreateEntity();
			LightComponent light1Comp;
			light1Comp.color = Vec3(0.8f, 0.2f, 0.1f);  // White light
			light1Comp.intensity = 2.0f;
			light1Comp.constant = 1.0f;
			light1Comp.linear = 0.05f;
			light1Comp.quadratic = 0.02f;
			Engine::AddComponent(light, light1Comp);

			TransformComponent light1Transform;
			light1Transform.position = Vec3(20 * i, 2 * y_height + 5, 20 * j);
			Engine::AddComponent(light, light1Transform);

			ColliderComponent collider;
			collider.type = ColliderComponent::Type::AABB;
			collider.box = bounds_component.bounds;
			Engine::AddComponent(e, collider);

			RigidbodyComponent rigid;
			rigid.useGravity = false;
			rigid.isKinematic = true;
			Engine::AddComponent(e, rigid);
		}
	}
}

