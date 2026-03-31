#include "ArrowSystem.h"

#include "AppSettings.h"
#include "FlowCommandPlaced.h"
#include "FlowFieldSystem.h"
#include "components/ArrowComponent.h"
#include "engine/Engine.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/MeshComponent.h"
#include <cmath>

void ArrowSystem::Init() {
	ComponentMask arrowMask;
	arrowMask.set(Engine::GetTypeID<ArrowComponent>());
	arrowMask.set(Engine::GetTypeID<TransformComponent>());
	addEntityView(arrowMask);

	Engine::GetSystem<FlowFieldSystem>().AddObserver(this);

	//to compute random phase
	m_rng = std::mt19937(std::random_device{}());
	m_phaseDist = std::uniform_real_distribution<float>(0.0f, 2.0f * PI);
}

void ArrowSystem::OnNotify(EventType type, const Event& payload) {
	if (type != EventType::FLOW_COMMAND_PLACED) {
		return;
	}
	//get the payload
	const FlowCommandPlaced* placedEvent = dynamic_cast<const FlowCommandPlaced*>(&payload);

	// iterate through every arrow
	for (const Entity& arrow: m_entityViews[0].dense()) {
		ArrowComponent& arrowComp = Engine::GetComponent<ArrowComponent>(arrow);
		TransformComponent& transComp = Engine::GetComponent<TransformComponent>(arrow);
		// find the changed/placed arrow
		if (placedEvent->x == arrowComp.x && placedEvent->z == arrowComp.z) {
			// if there is no change in the arrow direction, do nothing
			if (placedEvent->direction == arrowComp.direction) {
				return;
			}
			// change arrow direction or delete it (if direction is 0,0,0)
			else {
				if (placedEvent->direction != Vec3(0,0,0)) {
					const float yaw = std::atan2(-placedEvent->direction.x, -placedEvent->direction.z);
					transComp.rotation = Vec3(placedEvent->direction.y * PI / 2, yaw, 0);
					arrowComp.direction = placedEvent->direction;
					return;
				}
				else {
					Engine::DestroyEntity(arrow);
					return;
				}
			}
		}
	}

	//create a new arrow if an existing arrow doesnt exist already
	Entity newArrow = Engine::CreateEntity();
	// add arrow component
	ArrowComponent c_arrow;
	c_arrow.x = placedEvent->x;
	c_arrow.z = placedEvent->z;
	c_arrow.direction = placedEvent->direction;
	c_arrow.baseY = placedEvent->position.y + 2.0f;

	//compute a random phase so that we don't have all the arrows bobbing together
	c_arrow.phase = m_phaseDist(m_rng);
	Engine::AddComponent(newArrow, c_arrow);

	// add transform with correct direction
	TransformComponent t_arrow;
	t_arrow.position = placedEvent->position;
	t_arrow.position.y = c_arrow.baseY;
	const float yaw = std::atan2(-placedEvent->direction.x, -placedEvent->direction.z);
	t_arrow.rotation = Vec3(placedEvent->direction.y * PI / 2, yaw, 0);
	t_arrow.scale = Vec3(1,1,1);
	Engine::AddComponent(newArrow, t_arrow);

	// add mesh
	MeshComponent m_arrow;
	m_arrow.mesh = Engine::loadModel("./data/TestData/arrow.obj");
	m_arrow.colour = Vec3(1.0f, 1.0f, 1.0f);
	Engine::AddComponent(newArrow, m_arrow);

	// add bounds
	BoundsComponent b_arrow;
	b_arrow.bounds = m_arrow.mesh->bounds;
	Engine::AddComponent(newArrow, b_arrow);
}

void ArrowSystem::Update(float deltaTime) {
	const float deltaSeconds = deltaTime * 0.001f;
	m_timeSeconds += deltaSeconds;

	for (const Entity& arrow : m_entityViews[0].dense()) {
		ArrowComponent& arrowComp = Engine::GetComponent<ArrowComponent>(arrow);
		TransformComponent& transform = Engine::GetComponent<TransformComponent>(arrow);
		const float bob =
			std::sin(m_timeSeconds * m_floatSpeed + arrowComp.phase) * m_floatAmplitude;
		transform.position.y = arrowComp.baseY + bob;
	}
}

void ArrowSystem::Shutdown() {
	Engine::GetSystem<FlowFieldSystem>().RemoveObserver(this);
	m_timeSeconds = 0.0f;
}



