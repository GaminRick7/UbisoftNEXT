#include "AISystem.h"

#include "components/AIComponent.h"
#include "components/CrowdAgentComponent.h"
#include "components/CrowdGroupComponent.h"
#include "engine/components/BoundsComponent.h"
#include "engine/Engine.h"
#include "engine/components/TransformComponent.h"
#include "math/MathUtils.h"
#include "app.h"
#include <limits>
#include "systems/DroneDetected.h"
#include "systems/FlowFieldSystem.h"

void AISystem::Init() {
	ComponentMask mask;
	mask.set(Engine::GetTypeID<AIComponent>());
	mask.set(Engine::GetTypeID<TransformComponent>());
	addEntityView(mask);

	ComponentMask groupMask;
	groupMask.set(Engine::GetTypeID<CrowdGroupComponent>());
	groupMask.set(Engine::GetTypeID<TransformComponent>());
	groupMask.set(Engine::GetTypeID<BoundsComponent>());
	addEntityView(groupMask);

	ComponentMask agentMask;
	agentMask.set(Engine::GetTypeID<CrowdAgentComponent>());
	agentMask.set(Engine::GetTypeID<TransformComponent>());
	addEntityView(agentMask);
}

void AISystem::Update(float deltaTime) {
	const float deltaSeconds = deltaTime * 0.001f;
	const auto& controller = App::GetController();
	if (controller.CheckButton(App::BTN_LBUMPER)) {
		for (auto& entity: m_entityViews[0].dense()) {
			AIComponent& ai = Engine::GetComponent<AIComponent>(entity);
			ai.drawPath = !ai.drawPath;
		}
	}
	for (auto& entity: m_entityViews[0].dense()) {
		AIComponent& ai = Engine::GetComponent<AIComponent>(entity);

		const bool detected = checkForAgents(entity);
		if (ai.isHeldForDetection) {
			if (detected) {
				// start the no detectiuion timer
				ai.noDetectionTimer = 0.0f;
			} else {
				// update the noin detction timer
				ai.noDetectionTimer += deltaSeconds;

				//1.5 seconds passed without detection, the drone can move on
				if (ai.noDetectionTimer >= 1.5f) {
					ai.isHeldForDetection = false;
					ai.noDetectionTimer = 0.0f;
					ai.isMoving = true;

					// notify observers that the drone has cleared
					DroneDetected clearEvent;
					clearEvent.pos = ai.heldCellCenter;
					NotifyObservers(EventType::DRONE_CLEARED, clearEvent);
				}
			}
			continue;
		}
		if (ai.path.empty()) {
			continue;
		}
		// loop back
		if (ai.currentPathIndex >= ai.path.size()) {
			ai.currentPathIndex = 0;
		}
		if (!ai.isMoving) {
			continue;
		}
		TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);

		const Vec3 target = ai.path[ai.currentPathIndex];
		const Vec3 toTarget = target - transform.position;
		const float distance = toTarget.length();

		if (distance <= ai.arriveDistance) {
			ai.currentPathIndex++;
			ai.velocity = Vec3(0.0f, 0.0f, 0.0f);

			
			if (ai.stopOnArrival) {
				ai.isMoving = false;
				ai.stopOnArrival = false;
			}
			continue;
		}

		const Vec3 desiredDir = toTarget.normalized();
		float blend = ai.turnRate * deltaSeconds;
		if (blend > 1.0f) {
			blend = 1.0f;
		}
		Vec3 currentDir = ai.velocity.length() > 0.0001f ? ai.velocity.normalized() : desiredDir;
		Vec3 newDir = lerp(currentDir, desiredDir, blend);
		newDir.normalize();
		ai.velocity = newDir * ai.speed;
		transform.position = transform.position + ai.velocity * deltaSeconds;
		const Vec3 dir = ai.velocity.normalized();
		const float yaw = std::atan2(dir.x, dir.z);
		transform.rotation = Vec3(transform.rotation.x, yaw, 0.0f);
	}
}

void AISystem::Shutdown() {
}

bool AISystem::checkForAgents(Entity& entity) {
	TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
	AIComponent& ai = Engine::GetComponent<AIComponent>(entity);
	
	// use the flowww cell to build the world space bound sof that cell
	FlowFieldSystem& flowSystem = Engine::GetSystem<FlowFieldSystem>();
	const int flowWidth = flowSystem.GetFlowWidth();
	const int flowHeight = flowSystem.GetFlowHeight();
	const float cellSize = flowSystem.GetCellSize();
	const Vec3 origin = flowSystem.GetFlowOrigin();
	int droneCellX = -1;
	int droneCellZ = -1;
	if (flowWidth > 0 && flowHeight > 0 && cellSize > 0.0f) {
		const float relX = (transform.position.x - origin.x) / cellSize;
		const float relZ = (transform.position.z - origin.z) / cellSize;
		droneCellX = clampFloat(relX, 0.0f, flowWidth - 1);
		droneCellZ = clampFloat(relZ, 0.0f, flowHeight - 1);
	}

	// cell mins and maxes of the cell overlapping with drone
	const float cellMinX = origin.x + droneCellX * cellSize;
	const float cellMaxX = cellMinX + cellSize;
	const float cellMinZ = origin.z + droneCellZ * cellSize;
	const float cellMaxZ = cellMinZ + cellSize;
	const float cellMinY = transform.position.y - (cellSize * 0.5f);
	const float cellMaxY = transform.position.y + (cellSize * 0.5f);

	// make a bounds AABB
	const AABB cellBounds(
		Vec3(cellMinX, cellMinY, cellMinZ),
		Vec3(cellMaxX, cellMaxY, cellMaxZ)
	);

	// iterate throughall groups and check for overlap
	bool groupOverlap = false;
	for (const Entity& e : m_entityViews[1].dense()) {
			if (droneCellX < 0 || droneCellZ < 0 ||
			    !Engine::HasComponent<TransformComponent>(e) ||
			    !Engine::HasComponent<BoundsComponent>(e)) {
				continue;
			}
			const TransformComponent& crowdTransform = Engine::GetComponent<TransformComponent>(e);
			const BoundsComponent& crowdBounds = Engine::GetComponent<BoundsComponent>(e);

			// get the group's AABB
			const AABB crowdWorld =
				crowdBounds.bounds.computeWorldBounds(
					crowdTransform.position,
					crowdTransform.rotation,
					crowdTransform.scale
				);

			// check for intersection
			if (!crowdWorld.intersects(cellBounds)) {
				continue;
			}
		// set group overlap to be true
		groupOverlap = true;
		break;
	}

	if (!groupOverlap) {
		return false;
	}

	bool agentOverlap = false;
	int detectedGroupId = -1;
	for (const Entity& e : m_entityViews[2].dense()) {
		CrowdAgentComponent& agent = Engine::GetComponent<CrowdAgentComponent>(e);
		TransformComponent& agentTransform = Engine::GetComponent<TransformComponent>(e);
		float maxAbove = 0.5f;
		float maxBelow = cellSize * 0.5f;
		float yDelta = agentTransform.position.y - transform.position.y;
		if (yDelta > maxAbove || yDelta < -maxBelow) {
			continue;
		}
		float agentRelX = (agentTransform.position.x - origin.x) / cellSize;
		float agentRelZ = (agentTransform.position.z - origin.z) / cellSize;
		int agentCellX = clampFloat(agentRelX, 0.0f, flowWidth - 1);
		int agentCellZ = clampFloat(agentRelZ, 0.0f, flowHeight - 1);
		if (agentCellX == droneCellX && agentCellZ == droneCellZ) {
			agentOverlap = true;
			detectedGroupId = agent.groupId;
			break;
		}
	}

	if (!agentOverlap) {
		return false;
	}

	if (!ai.isHeldForDetection) {
		DroneDetected event;
		event.pos = transform.position;
		event.groupID = detectedGroupId;
		NotifyObservers(EventType::DRONE_DETECTED, event);
		//calculate the center of the cell so we can snap the drone to it
		Vec3 center(
			origin.x + (droneCellX + 0.5f) * cellSize,
			transform.position.y,
			origin.z + (droneCellZ + 0.5f) * cellSize
		);
		ai.heldCellCenter = center;
		ai.isHeldForDetection = true;
		ai.noDetectionTimer = 0.0f;
		transform.position = center;
		ai.velocity = Vec3(0.0f, 0.0f, 0.0f);
		ai.isMoving = false;
		ai.stopOnArrival = false;
	} else {
		// snap the drone to the center of the cell and stop from moveingdf
		transform.position = ai.heldCellCenter;
		ai.velocity = Vec3(0.0f, 0.0f, 0.0f);
		ai.isMoving = false;
	}
	return true;
}


