#include "FlowFieldSystem.h"

#include "app.h"
#include "components/FlowCellComponent.h"
#include "engine/Engine.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/ColliderComponent.h"
#include "engine/components/CameraComponent.h"
#include "engine/components/LightComponent.h"
#include "engine/graphics/Model.h"
#include "engine/components/MeshComponent.h"
#include "engine/components/TransformComponent.h"
#include "systems/FlowCommandPlaced.h"
#include "math/MathUtils.h"
#include <cmath>
void FlowFieldSystem::Init() {
	// entity view for camera
	ComponentMask cameraMask;
	cameraMask.set(Engine::GetTypeID<CameraComponent>());
	addEntityView(cameraMask);

	// entity view for flow cells
	ComponentMask flowMask;
	flowMask.set(Engine::GetTypeID<FlowCellComponent>());
	flowMask.set(Engine::GetTypeID<TransformComponent>());
	flowMask.set(Engine::GetTypeID<MeshComponent>());
	addEntityView(flowMask);

	m_hoverLightCreated = false;
}

void FlowFieldSystem::BuildFlowLevel(std::vector<int>& heightMap, int width, int length, int goalX, int goalZ) {
	// given a width x length heightmap, lets build a level!

	// store the level data in the system for easy access everywhere else
	m_flowWidth = width;
	m_flowLength = length;
	m_flowCellSize = 10.0f;
	m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);

	for (int z = 0; z < m_flowLength; ++z) {
		for (int x = 0; x < m_flowWidth; ++x) {
			// pull the height from the height map
			const int heightValue = heightMap[z * m_flowWidth + x];
			// create flow cell
			Entity cell = Engine::CreateEntity();

			const bool isGoalCell = (x == goalX && z == goalZ);

			// store the flow fields data in each cell so that it can be accessed by other systems
			// there's possible a better way of doing this but I did not want to call systems from other systems ;')
			FlowCellComponent flowComp;
			flowComp.x = x;
			flowComp.z = z;
			flowComp.gridWidth = m_flowWidth;
			flowComp.gridHeight = m_flowLength;
			flowComp.cellSize = m_flowCellSize;
			flowComp.cellHeight = heightValue;
			flowComp.origin = m_flowOrigin;
			flowComp.isGoal = isGoalCell;
			Engine::AddComponent(cell, flowComp);

			// reposition the cube to appear as a small plane that sits at the desired height
			TransformComponent cellTransform;
			cellTransform.position = Vec3(
				m_flowOrigin.x + (x + 0.5f) * m_flowCellSize,
				heightValue * m_flowCellSize,
				m_flowOrigin.z + (z + 0.5f) * m_flowCellSize
			);
			cellTransform.scale = Vec3(m_flowCellSize / 2, 0.1f, m_flowCellSize / 2);
			cellTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
			Engine::AddComponent(cell, cellTransform);

			// add mesh
			MeshComponent cellMesh;
			cellMesh.mesh = Engine::loadModel("./data/TestData/cube.obj");

			// make goal cells gold
			if (isGoalCell) {
				cellMesh.colour = Vec3(1.0f, 0.84f, 0.0f);
			} else {
				cellMesh.colour = Vec3(0.4f, 0.4f, 0.4f);
			}
			Engine::AddComponent(cell, cellMesh);

			// add bounds
			BoundsComponent cellBounds;
			cellBounds.bounds = cellMesh.mesh->bounds;
			Engine::AddComponent(cell, cellBounds);

			// specail goal cell creation logic to give it a light component
			if (isGoalCell) {
				Entity goalLight = Engine::CreateEntity();
				TransformComponent goalLightTransform;
				goalLightTransform.position = cellTransform.position + Vec3(0.0f, m_flowCellSize * 1.2f, 0.0f);
				goalLightTransform.scale = Vec3(1.0f, 1.0f, 1.0f);
				goalLightTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
				Engine::AddComponent(goalLight, goalLightTransform);

				LightComponent goalLightComponent;
				goalLightComponent.color = Vec3(1.0f, 0.84f, 0.0f);
				goalLightComponent.intensity = 5.0f;
				goalLightComponent.constant = 1.0f;
				goalLightComponent.linear = 0.14f;
				goalLightComponent.quadratic = 0.07f;
				Engine::AddComponent(goalLight, goalLightComponent);
			}

			if (heightValue > 0) {
				// if the height is greater than 0, lets ffill the gap from the ground to the flow cell with a block
				Entity heightCell = Engine::CreateEntity();

				//reposition + scale according to height
				TransformComponent heightTransform;
				heightTransform.position = Vec3(
					m_flowOrigin.x + (x + 0.5f) * m_flowCellSize,
					heightValue * (m_flowCellSize / 2.0f),
					m_flowOrigin.z + (z + 0.5f) * m_flowCellSize
				);
				heightTransform.scale = Vec3(
					m_flowCellSize / 2.0f,
					heightValue * (m_flowCellSize / 2.0f),
					m_flowCellSize / 2.0f
				);
				heightTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
				Engine::AddComponent(heightCell, heightTransform);

				// add mesh
				MeshComponent heightMesh;
				heightMesh.mesh = Engine::loadModel("./data/TestData/cube.obj");
				heightMesh.colour = Vec3(0.4f, 0.4f, 0.4f);
				Engine::AddComponent(heightCell, heightMesh);

				// add bounds
				BoundsComponent heightBounds;
				heightBounds.bounds = heightMesh.mesh->bounds;
				Engine::AddComponent(heightCell, heightBounds);

				// add collider
				ColliderComponent heightCollider;
				heightCollider.type = ColliderComponent::Type::AABB;
				heightCollider.box = heightBounds.bounds;
				Engine::AddComponent(heightCell, heightCollider);
			}
		}
	}

	// build the position -> flow cell vector/map
	buildFlowCellLookup();

	/// set the direction of the first cell to a direction so that the entities can start moving
	if (m_flowCellLookupBuilt && !m_flowCellLookup.empty()) {
		FlowCellComponent& firstCell = Engine::GetComponent<FlowCellComponent>(m_flowCellLookup.front());
		firstCell.direction = Vec3(0, 0, 1);
	}
}

void FlowFieldSystem::Update(float deltaTime) {
	// build/rebuild the flow cell map
	if (!m_flowCellLookupBuilt) {
		buildFlowCellLookup();
	}
	// set the previously hovered cell location
	int prevSelected = m_hover;
	handleInputs();

	// update the mesh colour of the selected/hovered cell
	for (Entity& flowcell : m_flowCellLookup) {
		FlowCellComponent& cellFlow = Engine::GetComponent<FlowCellComponent>(flowcell);
		MeshComponent& cellMesh = Engine::GetComponent<MeshComponent>(flowcell);
		// goal cell is gold
		if (cellFlow.isGoal) {
			cellMesh.colour = Vec3(1.0f, 0.84f, 0.0f);
			continue;
		}
		// reset the colour of the previously selected cell
		if (prevSelected != m_hover && cellFlow.z * m_flowWidth + cellFlow.x == prevSelected) {
			cellMesh.colour = Vec3(0.4f, 0.4f, 0.4f);
		}
		// update the selected cell to blue
		else if (cellFlow.z * m_flowWidth + cellFlow.x == m_selected) {
			cellMesh.colour = Vec3(0.2f, 0.2f, 0.6f);
		}
		// updtaed hovered cell to red
		else if (cellFlow.z * m_flowWidth + cellFlow.x == m_hover) {
			cellMesh.colour = Vec3(1.0f, 0.0f, 0.0f);
		}
	}

	updateLight();
}

void FlowFieldSystem::Shutdown() {
	m_hover = 0;
	m_selected = -1;
	m_hoverLight = Entity();
	m_hoverLightCreated = false;

	m_flowCellLookup.clear();
	m_flowCellLookupBuilt = false;
	m_flowWidth = 0;
	m_flowLength = 0;
	m_flowCellSize = 1.0f;
	m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);
}

// Not to be used in game logic
// only created this so that we can pre place commands onm the main menu level
bool FlowFieldSystem::PlaceCommand(int x, int z, const Vec3& direction) {

	//fail conditions
	if (!m_flowCellLookupBuilt) {
		buildFlowCellLookup();
	}
	if (!m_flowCellLookupBuilt) {
		return false;
	}
	const Entity& cellEntity = m_flowCellLookup[z * m_flowWidth + x];

	// get the flow cell's components
	FlowCellComponent& cellFlow = Engine::GetComponent<FlowCellComponent>(cellEntity);
	TransformComponent& cellTransform = Engine::GetComponent<TransformComponent>(cellEntity);
	cellFlow.direction = direction;

	// place the direction and notify away!
	FlowCommandPlaced event;
	event.x = cellFlow.x;
	event.z = cellFlow.z;
	event.position = cellTransform.position;
	event.direction = direction;
	NotifyObservers(EventType::FLOW_COMMAND_PLACED, event);
	return true;
}

void FlowFieldSystem::handleInputs() {
	// get the active camera
	CameraComponent* activeCamera = nullptr;
	auto& cameraEntities = m_entityViews[0].dense();
	for (const Entity& entity : cameraEntities) {
		CameraComponent& camera = Engine::GetComponent<CameraComponent>(entity);
		if (camera.isActive) {
			activeCamera = &camera;
			break;
		}
	}

	// calculate the camera's forward vector
	const Vec3 worldUp(0.0f, 1.0f, 0.0f);
	Vec3 forward(0.0f, 0.0f, 1.0f);
	forward = activeCamera->target - activeCamera->position;
	forward.y = 0.0f;
	forward.normalize();

	// ... and right vector
	Vec3 right = forward.cross(worldUp).normalized();

	// if we are hovering and not selecting
	if (m_selected == -1) {
		if (App::GetController().CheckButton(App::BTN_DPAD_UP)) {
			stepHoverFromDir(forward);
		}
		if (App::GetController().CheckButton(App::BTN_DPAD_DOWN)) {
			stepHoverFromDir(forward * -1.0f);
		}
		if (App::GetController().CheckButton(App::BTN_DPAD_RIGHT)) {
			stepHoverFromDir(right);
		}
		if (App::GetController().CheckButton(App::BTN_DPAD_LEFT)) {
			stepHoverFromDir(right * -1.0f);
		}
	} else { // we have a selected cell
		// get the selected cell's components
		const Entity& e = m_flowCellLookup[m_selected];
		FlowCellComponent& cellFlow = Engine::GetComponent<FlowCellComponent>(e);
		TransformComponent& cellTransform = Engine::GetComponent<TransformComponent>(e);
		bool commandPlaced = false;
		Vec3 commandDirection(0.0f, 0.0f, 0.0f);

		// get the command direction relative to the camera
		// for each of these cases, we pass the forward, right, left, and back vector from the camera
		// getAxisVectorFromDir finds most dominant direction (x or z) in the direction we want to go in
		// and essentially drops the other term, this gives the command direction in world space
		if (App::GetController().CheckButton(App::BTN_DPAD_LEFT)) {
			commandDirection = getAxisVectorFromDir(right * -1.0f);
			commandPlaced = true;
		}
		if (App::GetController().CheckButton(App::BTN_DPAD_RIGHT)) {
			commandDirection = getAxisVectorFromDir(right);
			commandPlaced = true;
		}
		if (App::GetController().CheckButton(App::BTN_DPAD_UP)) {
			commandDirection = getAxisVectorFromDir(forward);
			commandPlaced = true;
		}
		if (App::GetController().CheckButton(App::BTN_DPAD_DOWN)) {
			commandDirection = getAxisVectorFromDir(forward * -1.0f);
			commandPlaced = true;
		}
		if (App::GetController().CheckButton(App::BTN_Y)) {
			// up command does not need to be relative to camera
			commandDirection = Vec3(0, 1, 0);
			commandPlaced = true;
		}
		if (App::GetController().CheckButton(App::BTN_X)) {
			// clear cell command does not need to be relative to camera
			commandDirection = Vec3(0, 0, 0);
			commandPlaced = true;
		}

		// notify observers/systems that a command has been placed
		if (commandPlaced) {
			cellFlow.direction = commandDirection;
			m_selected = -1;
			FlowCommandPlaced event;
			event.x = cellFlow.x;
			event.z = cellFlow.z;
			event.position = cellTransform.position;
			event.direction = commandDirection;
			NotifyObservers(EventType::FLOW_COMMAND_PLACED, event);
		}
	}
	// select the the hovered cell
	if (App::GetController().CheckButton(App::BTN_A)) {
		m_selected = m_hover;
	}

	// deselect
	if (App::GetController().CheckButton(App::BTN_B)) {
		m_selected = -1;
	}
}

void FlowFieldSystem::buildFlowCellLookup() {

	m_flowCellLookup.clear();
	m_flowWidth = 0;
	m_flowLength = 0;
	m_flowCellSize = 1.0f;
	m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);
	if (m_entityViews.size() <= 1) {
		m_flowCellLookupBuilt = false;
		return;
	}

	auto& flowCells = m_entityViews[1].dense();
	if (flowCells.empty()) {
		m_flowCellLookupBuilt = false;
		return;
	}

	const FlowCellComponent& sampleCell = Engine::GetComponent<FlowCellComponent>(flowCells.front());
	if (sampleCell.gridWidth <= 0 || sampleCell.gridHeight <= 0 || sampleCell.cellSize <= 0.0f) {
		m_flowCellLookupBuilt = false;
		return;
	}

	m_flowWidth = sampleCell.gridWidth;
	m_flowLength = sampleCell.gridHeight;
	m_flowCellSize = sampleCell.cellSize;
	m_flowOrigin = sampleCell.origin;
	m_flowCellLookup.assign(m_flowWidth * m_flowLength, Entity());
	for (const Entity& cellEntity : flowCells) {
		const FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(cellEntity);
		if (cell.x < 0 || cell.z < 0 || cell.x >= m_flowWidth || cell.z >= m_flowLength) {
			continue;
		}
		const int idx = cell.z * m_flowWidth + cell.x;
		m_flowCellLookup[idx] = cellEntity;
	}
	m_flowCellLookupBuilt = true;
}

void FlowFieldSystem::stepHoverFromDir(const Vec3& dir) {
	int dx = 0;
	int dz = 0;
	// if x dominates, set dx to 1, otherwise set dz to 1
	// use negatives for the opposite direction
	if (std::fabs(dir.x) >= std::fabs(dir.z)) {
		if (dir.x > 0.0f) {
			dx = 1;
		} else if (dir.x < 0.0f) {
			dx = -1;
		}
	} else {
		if (dir.z > 0.0f) {
			dz = 1;
		} else if (dir.z < 0.0f) {
			dz = -1;
		}
	}
	const int x = m_hover % m_flowWidth;
	const int z = m_hover / m_flowWidth;

	// calculate new hover position
	const int newX = x + dx;
	const int newZ = z + dz;

	// ensure the new hover position is with in the bounds of the map
	if (newX >= 0 && newX < m_flowWidth && newZ >= 0 && newZ < m_flowLength) {
		m_hover = newZ * m_flowWidth + newX;
		NotifyObservers(EventType::FLOW_GRID_MOVED, Event());
	}
}

Vec3 FlowFieldSystem::getAxisVectorFromDir(const Vec3& dir) {
	if (std::fabs(dir.x) < 0.0001f && std::fabs(dir.z) < 0.0001f) {
		return Vec3(0.0f, 0.0f, 0.0f);
	}
	if (std::fabs(dir.x) >= std::fabs(dir.z)) {
		if (dir.x > 0.0f) {
			return Vec3(1.0f, 0.0f, 0.0f);
		}
		if (dir.x < 0.0f) {
			return Vec3(-1.0f, 0.0f, 0.0f);
		}
		return Vec3(0.0f, 0.0f, 0.0f);
	}
	if (dir.z > 0.0f) {
		return Vec3(0.0f, 0.0f, 1.0f);
	}
	if (dir.z < 0.0f) {
		return Vec3(0.0f, 0.0f, -1.0f);
	}
	return Vec3(0.0f, 0.0f, 0.0f);
}

void FlowFieldSystem::updateLight() {
	// keep a hover light above the current cell
	// create the light if it hasn't been created yet
	if (!m_hoverLightCreated) {
		m_hoverLight = Engine::CreateEntity();
		TransformComponent lightTransform;
		lightTransform.position = Vec3(0.0f, 0.0f, 0.0f);
		lightTransform.scale = Vec3(1.0f, 1.0f, 1.0f);
		lightTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
		Engine::AddComponent(m_hoverLight, lightTransform);

		LightComponent light;
		light.color = Vec3(1.0f, 0.0f, 0.0f);
		light.intensity = 5.0f;
		light.constant = 1.0f;
		light.linear = 0.14f;
		light.quadratic = 0.07f;
		Engine::AddComponent(m_hoverLight, light);
		m_hoverLightCreated = true;
	} else {
		// light should be red if the cell is hovered and blue if selected
		LightComponent& light = Engine::GetComponent<LightComponent>(m_hoverLight);
		if (m_selected == m_hover && m_selected >= 0) {
			light.color = Vec3(0.2f, 0.4f, 1.0f);
		} else {
			light.color = Vec3(1.0f, 0.0f, 0.0f);
		}
	}

	// move light to the hovered cell
	const Entity& hoverCell = m_flowCellLookup[m_hover];
	const TransformComponent& cellTransform = Engine::GetComponent<TransformComponent>(hoverCell);
	TransformComponent& lightTransform = Engine::GetComponent<TransformComponent>(m_hoverLight);
	lightTransform.position = cellTransform.position + Vec3(0.0f, m_flowCellSize * 1.2f, 0.0f);
}