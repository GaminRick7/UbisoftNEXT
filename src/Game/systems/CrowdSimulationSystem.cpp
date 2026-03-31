#include "CrowdSimulationSystem.h"

#include "components/CrowdAgentComponent.h"
#include "components/CrowdGroupComponent.h"
#include "components/FlowCellComponent.h"
#include "engine/Engine.h"
#include "engine/components/BoundsComponent.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "app.h"
#include "DroneDetected.h"
#include "AI/AISystem.h"
#include "math/MathUtils.h"

void CrowdSimulationSystem::Init() {
    // mask for individual agents
	ComponentMask mask;
	mask.set(Engine::GetTypeID<CrowdAgentComponent>());
	mask.set(Engine::GetTypeID<TransformComponent>());
	addEntityView(mask);

    // mask for crowd groups
	ComponentMask groupMask;
	groupMask.set(Engine::GetTypeID<CrowdGroupComponent>());
	groupMask.set(Engine::GetTypeID<TransformComponent>());
	groupMask.set(Engine::GetTypeID<BoundsComponent>());
	addEntityView(groupMask);

	// mask for flow cells
	ComponentMask flowMask;
	flowMask.set(Engine::GetTypeID<FlowCellComponent>());
	flowMask.set(Engine::GetTypeID<TransformComponent>());
	addEntityView(flowMask);

	// setup spatial hash
	// it same origin as flow glow gridbut different cell size)
	m_hashOrigin = Vec3(0.0f, 0.0f, 0.0f);
	const float hashWorldWidth = 100.0f;
	const float hashWorldHeight = 100.0f;

    // calculate teh number of hash cells needed
	m_hashWidth = (std::max)(1, static_cast<int>(std::ceil(hashWorldWidth / m_hashCellSize)));
	m_hashHeight = (std::max)(1, static_cast<int>(std::ceil(hashWorldHeight / m_hashCellSize)));
	m_hashBuckets.resize(m_hashWidth * m_hashHeight);
	m_freeGroupIds.reserve(m_hashWidth * m_hashHeight);

	Engine::GetSystem<AISystem>().AddObserver(this);
}

void CrowdSimulationSystem::SetMaxSpeed(float maxSpeed) {
	if (maxSpeed <= 0.0f) {
		return;
	}
	m_maxSpeed = maxSpeed;
}

void CrowdSimulationSystem::Update(float deltaTime) {
	//convert to seconds
	const float deltaSeconds = deltaTime * 0.001f;

	// build flow cell lookup if not built already
	if (!m_flowCellLookupBuilt) {
		buildFlowCellLookup();
	}

	//rebuild the spatial hash based on updated entity positions
	rebuildSpatialHash();

	// if we introduced new groups, lets rebuild the groupID -> group look up table
	if (!m_groupEntitiesBuilt) {
		buildGroupEntities();
	}

	// since the group bounds change every frame, we must recompute them
	// lets place dummy bound mins and maxes before recalculating
	std::vector<Vec3> groupMin;
	std::vector<Vec3> groupMax;
	std::vector<bool> groupHasData;
	if (!m_groupEntities.empty()) {
        // add placeholder values for each group's bounds
		const size_t groupCount = m_groupEntities.size();
		if (m_groupCounts.size() < groupCount) {
			m_groupCounts.resize(groupCount, 0);
		}
		groupMin.assign(
			groupCount,
			Vec3(
				(std::numeric_limits<float>::max)(),
				(std::numeric_limits<float>::max)(),
				(std::numeric_limits<float>::max)()
			)
		);
		groupMax.assign(
			groupCount,
			Vec3(
				std::numeric_limits<float>::lowest(),
				std::numeric_limits<float>::lowest(),
				std::numeric_limits<float>::lowest()
			)
		);
		groupHasData.assign(groupCount, false);
	}

	// let's iterate through each individual agent
	for (const Entity& entity : m_entityViews[0].dense()) {
		CrowdAgentComponent& agent = Engine::GetComponent<CrowdAgentComponent>(entity);
		TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);

        // get the agents old flow flow
		Vec3 flow;
		const Vec3& oldFlow = agent.flow;
		Vec3 newFlow(0.0f, 0.0f, 0.0f);
		float groundY = 0.0f;
		Vec3 cellCenter(0.0f, 0.0f, 0.0f);
		bool isGoalCell = false;

		// lets get all the flow cell data using sampleFlowCell
		if (sampleFlowCell(transform, newFlow, groundY, cellCenter, isGoalCell) &&
		    newFlow.length() > 0.0001f) {
			// take the agent's incoming velocity
			Vec3 incoming = agent.velocity;
			incoming.y = 0.0f;
			if (incoming.length() < 0.0001f) {
				incoming = oldFlow;
				incoming.y = 0.0f;
			}
			// flow commands should only start affecting you in the middle strip of the the cell 
			// perpendicular to where the agent is coming from
			if (!isWithinFlowStrip(transform.position, cellCenter, incoming)) {
				newFlow = Vec3(0.0f, 0.0f, 0.0f);
				isGoalCell = false;
			}
		}

		if (isGoalCell && !agent.reachedGoal) {
			agent.reachedGoal = true;
			m_goalReachedCount++;
			if (m_goalReachedCount >= 110 && !m_goalWinPrinted) {
				m_goalWinPrinted = true;
				NotifyObservers(EventType::GOAL_COUNT_REACHED, Event());
			}
		}

		// if we encounter a new flow cell with a different direction, follow that flow
		// no flow squares are 0 vectors -> when we encounter these, keep the old flow

		// SPECIAL we remove the old flow completely and send the agents straight up if its a goal cell
		if (isGoalCell) {
			flow = Vec3(0.0f, 1.0f, 0.0f);
			agent.flow = flow;
		} else if (newFlow.length() < 0.0001f) {
			flow = oldFlow;
		}
		else {
			// positive y component indicates that we have a leap cell so we keep the previous x and z but change y
			if (newFlow.y > 0.001f) {
				flow = Vec3(oldFlow.x, newFlow.y, oldFlow.z);
			}
			else {
				flow = newFlow;
			}
			agent.flow = flow;
		}

		// calculate the separation between this agent and nearby agents
		Vec3 separation = calculateSeparation(entity, transform);

        // get the cohesion to group
        // we keep grouped entities together to reduce the physical size of each group
        // this way the bounds components stays minimized, improving frustum culling
		Vec3 cohesion = calculateGroupCohesion(entity, transform);

		Vec3 desiredDir = flow + separation * m_separationStrength + cohesion * m_cohesionStrength;
		if (isGoalCell) {
			desiredDir = Vec3(0.0f, 1.0f, 0.0f);
		}
		if (desiredDir.length() > 0.0001f) {
			desiredDir.normalize();
		}

        // blend controls how much the agent’s velocity is allowed to rotate toward the desired direction
		// - > how fast we change directions
		const Vec3 targetVelocity = desiredDir * m_maxSpeed;
		float blend = m_response * deltaSeconds;
		if (blend > 1.0f) {
			blend = 1.0f;
		}
        
        // use blend to steer on XZ while hovering on Y when inside a lift cell
		Vec3 planarVelocity(agent.velocity.x, 0.0f, agent.velocity.z);
		Vec3 planarTarget(targetVelocity.x, 0.0f, targetVelocity.z);
		planarVelocity = lerp(planarVelocity, planarTarget, blend);

		//
		Vec3 newVelocity;
		Vec3 newPosition;

		// igmore the hover if we are at goal cell
		if (isGoalCell) {
			newVelocity = Vec3(0.0f, m_maxSpeed, 0.0f);
			newPosition = transform.position + newVelocity * deltaSeconds;
		} 
		// otherwise lets hover!!!!
		else {
			updateHoverMotion(newFlow, groundY, deltaSeconds, planarVelocity, transform, agent, newVelocity, newPosition);
		}

		
		// get the kill cell index
		const int killCellIndex = getFlowCellIndex(newPosition);
		if (killCellIndex >= 0 && killCellIndex < m_flowCellLookup.size()) {

			//get the flow cell entity
			const Entity killCellEntity = m_flowCellLookup[killCellIndex];
			if (killCellEntity.id != UINT32_MAX &&
			    Engine::HasComponent<FlowCellComponent>(killCellEntity) &&
			    Engine::HasComponent<TransformComponent>(killCellEntity)) {
				const FlowCellComponent& killCell =
					Engine::GetComponent<FlowCellComponent>(killCellEntity);
				// if it is a kill cell, apply same striup as the flow strip and destroy agents in that strip
				if (killCell.isKill) {
					const TransformComponent& killCellTransform =
						Engine::GetComponent<TransformComponent>(killCellEntity);
					const float killCellTopY = killCellTransform.position.y + m_flowCellSize;
					const Vec3 killCellCenter(
						m_flowOrigin.x + (killCell.x + 0.5f) * m_flowCellSize,
						0.0f,
						m_flowOrigin.z + (killCell.z + 0.5f) * m_flowCellSize
					);
					Vec3 incoming = agent.velocity;
					incoming.y = 0.0f;
					if (newPosition.y <= killCellTopY &&
					    isWithinFlowStrip(newPosition, killCellCenter, incoming)) {
						DecrementGroupCount(agent.groupId);
						Engine::DestroyEntity(entity);
						continue;
					}
				}
			}
		}

		// check if the new position would hit a wall
		// destroy the entityty if it does
		if (wouldHitWall(newPosition, groundY, newPosition.y)) {
			Engine::DestroyEntity(entity);
		}

		agent.velocity = newVelocity;
		transform.position = newPosition;
		agent.position = transform.position;

		// Remove agents that leave the octree world bounds.
		if (transform.position.x < 0.0f || transform.position.x > 100.0f ||
		    transform.position.y < -25.0f || transform.position.y > 100.0f ||
		    transform.position.z < 0.0f || transform.position.z > 100.0f) {
			DecrementGroupCount(agent.groupId);
			Engine::DestroyEntity(entity);
			continue;
		}

        // update the groups bounds
		if (agent.groupId >= 0 && agent.groupId < groupMin.size()) {
			const int index = (agent.groupId);
			groupHasData[index] = true;
			groupMin[index].x = (std::min)(groupMin[index].x, transform.position.x);
			groupMin[index].y = (std::min)(groupMin[index].y, transform.position.y);
			groupMin[index].z = (std::min)(groupMin[index].z, transform.position.z);
			groupMax[index].x = (std::max)(groupMax[index].x, transform.position.x);
			groupMax[index].y = (std::max)(groupMax[index].y, transform.position.y);
			groupMax[index].z = (std::max)(groupMax[index].z, transform.position.z);
		}
	}

	for (size_t i = 0; i < m_groupEntities.size(); ++i) {
		if (!groupHasData[i]) {
			continue;
		}
		const Entity groupEntity = m_groupEntities[i];
		if (groupEntity.id == UINT32_MAX) {
			continue;
		}
		if (!Engine::HasComponent<TransformComponent>(groupEntity)) {
			m_groupEntities[i] = Entity();
			MarkGroupsDirty();
			continue;
		}

		const Vec3 center = (groupMin[i] + groupMax[i]) * 0.5f;
		TransformComponent& groupTransform = Engine::GetComponent<TransformComponent>(groupEntity);
		groupTransform.position = center;

		BoundsComponent& groupBounds = Engine::GetComponent<BoundsComponent>(groupEntity);
		const Vec3 localMin = groupMin[i] - center;
		const Vec3 localMax = groupMax[i] - center;
		groupBounds.bounds = AABB(localMin, localMax);
	}

	// Remove empty groups and recycle their IDs.
	const size_t groupCount = m_groupEntities.size();
	for (size_t i = 0; i < groupCount; ++i) {
		const Entity groupEntity = m_groupEntities[i];
		if (groupEntity.id == UINT32_MAX) {
			continue;
		}
		if (!Engine::HasComponent<CrowdGroupComponent>(groupEntity)) {
			m_groupEntities[i] = Entity();
			MarkGroupsDirty();
			continue;
		}
		if (i < m_groupCounts.size() && m_groupCounts[i] <= 0) {
			Engine::DestroyEntity(groupEntity);
			ReleaseGroupId(i);
			m_groupEntities[i] = Entity();
			MarkGroupsDirty();
		}
	}
}

void CrowdSimulationSystem::Shutdown() {
	m_maxSpeed = 6.0f;
	m_flowCellLookup.clear();
	m_flowCellLookupBuilt = false;
	m_flowWidth = 0;
	m_flowHeight = 0;
	m_flowCellSize = 1.0f;
	m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);
	m_pendingKillCellPositions.clear();
	m_pendingClearCellPositions.clear();
	m_goalReachedCount = 0;
	m_goalWinPrinted = false;

	m_hashBuckets.clear();
	m_hashWidth = 0;
	m_hashHeight = 0;
	m_hashOrigin = Vec3(0.0f, 0.0f, 0.0f);

	m_groupEntities.clear();
	m_groupEntitiesBuilt = false;
	m_freeGroupIds.clear();
	m_freeGroupTop = 0;
	m_nextGroupId = 0;
	m_groupCounts.clear();
}

void CrowdSimulationSystem::buildFlowCellLookup() {
	m_flowCellLookup.clear();
	if (m_entityViews.size() <= 2) {
		m_flowCellLookupBuilt = false;
		return;
	}

	auto& flowCells = m_entityViews[2].dense();
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
	m_flowHeight = sampleCell.gridHeight;
	m_flowCellSize = sampleCell.cellSize;
	m_flowOrigin = sampleCell.origin;
	m_flowCellLookup.assign(m_flowWidth * m_flowHeight, Entity());
	for (const Entity& cellEntity : flowCells) {
		const FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(cellEntity);
		if (cell.x < 0 || cell.z < 0 || cell.x >= m_flowWidth || cell.z >= m_flowHeight) {
			continue;
		}
		const int idx = cell.z * m_flowWidth + cell.x;
		m_flowCellLookup[idx] = cellEntity;
	}
	m_flowCellLookupBuilt = true;
	if (!m_pendingKillCellPositions.empty()) {

		//get the kpending kill cell positions and then mark the cells has kill cells in the actual
		// flow component >:)
		for (const Vec3& position : m_pendingKillCellPositions) {
			const int idx = getFlowCellIndex(position);
			if (idx >= 0 && idx < m_flowCellLookup.size()) {
				const Entity killCellEntity = m_flowCellLookup[idx];
				if (killCellEntity.id != UINT32_MAX &&
				    Engine::HasComponent<FlowCellComponent>(killCellEntity)) {
					FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(killCellEntity);
					cell.isKill = true;
				}
			}
		}
		m_pendingKillCellPositions.clear();
	}

	// get the pending clear kill cell positions and mark them as not kill ;-;
	if (!m_pendingClearCellPositions.empty()) {
		for (const Vec3& position : m_pendingClearCellPositions) {
			const int idx = getFlowCellIndex(position);
			if (idx >= 0 && idx < m_flowCellLookup.size()) {
				const Entity killCellEntity = m_flowCellLookup[idx];
				if (killCellEntity.id != UINT32_MAX &&
				    Engine::HasComponent<FlowCellComponent>(killCellEntity)) {
					FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(killCellEntity);
					cell.isKill = false;
				}
			}
		}
		m_pendingClearCellPositions.clear();
	}
}

Vec3 CrowdSimulationSystem::calculateSeparation(const Entity& entity, const TransformComponent& transform) const {
	Vec3 separation(0.0f, 0.0f, 0.0f);

	// convert the position to an index for hash buckets
	const float posX = (transform.position.x - m_hashOrigin.x) / m_hashCellSize;
	const float posZ = (transform.position.z - m_hashOrigin.z) / m_hashCellSize;
	const int cellX = (std::min)((std::max)(static_cast<int>(posX), 0), m_hashWidth - 1);
	const int cellZ = (std::min)((std::max)(static_cast<int>(posZ), 0), m_hashHeight - 1);


	const float sepRadiusSq = m_separationRadius * m_separationRadius;

	// 3x3 neighborhood around the current cell
	// if 0,0 is the current cell there are 8 other cellss
	const int neighborOffsets[9][2] = {
		{-1, -1}, {0, -1}, {1, -1},
		{-1,  0}, {0,  0}, {1,  0},
		{-1,  1}, {0,  1}, {1,  1},
	};

    // calculate the separation with every agent in the 3x3 neighbourhood within the separation radius
	for (const auto& offset : neighborOffsets) {
        // find cell position
		const int nx = cellX + offset[0];
		const int nz = cellZ + offset[1];
		if (nx < 0 || nz < 0 || nx >= m_hashWidth || nz >= m_hashHeight) {
			continue;
		}

        // get cell's index in the hash
		const int ncell = nz * m_hashWidth + nx;

        // calculate the separation to each entity in its cell
		for (const Entity& other : m_hashBuckets[ncell]) {
			if (other.id == entity.id) {
				continue;
			}
			const TransformComponent& otherTransform = Engine::GetComponent<TransformComponent>(other);

			const Vec3 offsetVec = transform.position - otherTransform.position;
			const float distSq = offsetVec.dot(offsetVec);

			// additional constraint that we should only sepaprate entities in the separation radius
			if (distSq < sepRadiusSq) {
				//separation should take into account inverse square of distance
				separation = separation + (offsetVec * (1.0f / std::sqrt(distSq)));
			}
		}
	}

	return separation;
}

Vec3 CrowdSimulationSystem::calculateGroupCohesion(const Entity& entity, const TransformComponent& transform) const {
	Vec3 cohesion(0.0f, 0.0f, 0.0f);

	const CrowdAgentComponent& agent = Engine::GetComponent<CrowdAgentComponent>(entity);
	const float cohesionRadiusSq = m_cohesionRadius * m_cohesionRadius;
	if (agent.groupId >= 0 && agent.groupId < m_groupEntities.size()) {
        // get the group's transform
		const Entity groupEntity = m_groupEntities[agent.groupId];
		if (Engine::HasComponent<TransformComponent>(groupEntity)) {
			const TransformComponent& groupTransform = Engine::GetComponent<TransformComponent>(groupEntity);

            //get the vector to the groups ceneter
			Vec3 toGroupCenter = groupTransform.position - transform.position;
			toGroupCenter.y = 0.0f;
			const float distSq = toGroupCenter.dot(toGroupCenter);

            // move entities in the radius closer to group center
			if (distSq > cohesionRadiusSq) {
				cohesion = toGroupCenter * (1.0f / std::sqrt(distSq));
			}
		}
	}

	return cohesion;
}

bool CrowdSimulationSystem::sampleFlowCell(const TransformComponent& transform, Vec3& outFlow, float& outGroundY, Vec3& outCellCenter, bool& outIsGoal) const {
	if (m_flowWidth <= 0 || m_flowHeight <= 0 || m_flowCellSize <= 0.0f || m_flowCellLookup.empty()) {
		return false;
	}

	// convert from position to index
	const float relX = (transform.position.x - m_flowOrigin.x) / m_flowCellSize;
	const float relZ = (transform.position.z - m_flowOrigin.z) / m_flowCellSize;
	const int x = clampFloat(relX, 0.0f, m_flowWidth - 1);
	const int z = clampFloat(relZ, 0.0f, m_flowHeight - 1);
	const int idx = z * m_flowWidth + x;
	if (idx < 0 || idx >= m_flowCellLookup.size()) {
		return false;
	}

	// get the associataed flow cell with the position
	const Entity cellEntity = m_flowCellLookup[idx];
	const FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(cellEntity);
	const TransformComponent& cellTransform = Engine::GetComponent<TransformComponent>(cellEntity);

	// store the flow direction + cell center! we need these later
	outIsGoal = cell.isGoal;
	if (outIsGoal) {
		outFlow = Vec3(0.0f, 1.0f, 0.0f);
	} else {
		outFlow = cell.direction;
	}
	outGroundY = cellTransform.position.y + 0.6f;
	outCellCenter = Vec3( m_flowOrigin.x + (x + 0.5f) * m_flowCellSize, 0.0f, m_flowOrigin.z + (z + 0.5f) * m_flowCellSize
	);
	return true;
}

bool CrowdSimulationSystem::isWithinFlowStrip(const Vec3& agentPosition, const Vec3& cellCenter, const Vec3& incoming) const {
	Vec3 incomingPlanar = incoming;
	incomingPlanar.y = 0.0f;
	if (incomingPlanar.length() <= 0.0001f) {
		return true;
	}
	incomingPlanar.normalize();

	//get the direction vetor from agent to cell center 
	const Vec3 toCenter(agentPosition.x - cellCenter.x, 0.0f, agentPosition.z - cellCenter.z);

	// abs dot product with incoming
	// this is the perpendicular distance from the agent
	//  to a line through the cell center that is perpendicular to the agent's motion
	// i swear it makes more sense with a diagram :')
	const float distToStrip = std::fabs(toCenter.dot(incomingPlanar));
	const float stripHalfWidth = m_flowCellSize * m_flowStripHalfWidthRatio;
	return distToStrip <= stripHalfWidth;
}

void CrowdSimulationSystem::updateHoverMotion(const Vec3& newFlow, float groundY, float deltaSeconds, const Vec3& planarVelocity,
	const TransformComponent& transform, CrowdAgentComponent& agent, Vec3& outVelocity,Vec3& outPosition) const {
	
	// trigger hover if we find flow of positive y
	const bool hoverTriggered =
		newFlow.y > 0.001f && agent.hoverDistanceRemaining <= 0.0f;
	// set the remaining distance and hover base height (height the hover starts at)
	if (hoverTriggered) {
		agent.hoverDistanceRemaining = m_flowCellSize * 2.3f;
		agent.hoverBaseY = groundY;
	}

	// keep the x/z speed constant througout the hovering
	const float planarSpeed =
		std::sqrt(planarVelocity.x * planarVelocity.x + planarVelocity.z * planarVelocity.z);
	
	// update remaining distance based on the planar speed
	if (agent.hoverDistanceRemaining > 0.0f) {
		agent.hoverDistanceRemaining -= planarSpeed * deltaSeconds;
		if (agent.hoverDistanceRemaining < 0.0f) {
			agent.hoverDistanceRemaining = 0.0f;
		}
	}

	const bool isHovering = agent.hoverDistanceRemaining > 0.0f;
	outVelocity = Vec3(planarVelocity.x, agent.velocity.y, planarVelocity.z);
	outPosition = transform.position + outVelocity * deltaSeconds;

	if (isHovering) {
		const float totalHoverDistance = m_flowCellSize * 2.3f;
		// convert progress into a standard 0-1 value
		const float progress =
			totalHoverDistance > 0.0f ? (1.0f - (agent.hoverDistanceRemaining / totalHoverDistance)) : 1.0f;
		// let's model the leap using a sinusoidal arc!
		const float arc =
			std::sin(progress * PI) * 15.0f;

		// the arc models position not velocity, results in a cleaner leap
		outPosition.y = agent.hoverBaseY + arc;
		outVelocity.y = 0.0f;
		// if we are clipping below the ground, means we have landed on a platform
		// so we stop the sine curve
		if (groundY >= outPosition.y - 0.001f) {
			outPosition.y = groundY;
			agent.hoverDistanceRemaining = 0.0f;
		}
	} else {
		// only apply gravity when we are not hovering
		outVelocity.y += m_gravity * deltaSeconds;
		outPosition.y += outVelocity.y * deltaSeconds;
		if (outPosition.y < groundY) {
			outPosition.y = groundY;
			if (outVelocity.y < 0.0f) {
				outVelocity.y = 0.0f;
			}
		}
	}
}

void CrowdSimulationSystem::buildGroupEntities() {
	// scan all groups and find the max group id
	int maxGroupId = -1;
	for (const Entity& groupEntity : m_entityViews[1].dense()) {
		const CrowdGroupComponent& group = Engine::GetComponent<CrowdGroupComponent>(groupEntity);
		if (group.groupId > maxGroupId) {
			maxGroupId = group.groupId;
		}
	}

	// no groups
	if (maxGroupId < 0) {
		return;
	}

	const int groupCount = maxGroupId + 1;
	// lets resize the the lookup table to match the group id
	m_groupEntities.resize(groupCount);

	// iterate through the group entities and store each group entity in the array at its groupId index
	// this will give us O(1) access later! :D
	for (const Entity& groupEntity : m_entityViews[1].dense()) {
		const CrowdGroupComponent& group = Engine::GetComponent<CrowdGroupComponent>(groupEntity);
		if (group.groupId >= 0 && group.groupId < groupCount) {
			m_groupEntities[group.groupId] = groupEntity;
		}
	}

	m_groupEntitiesBuilt = true;
}

int CrowdSimulationSystem::AcquireGroupId() {

    //try to get a group id deom the free group ids
	if (m_freeGroupTop > 0) {
		m_freeGroupTop -= 1;
		return m_freeGroupIds[m_freeGroupTop];
	}

    // otherwise, create a new id
	return m_nextGroupId++;
}

void CrowdSimulationSystem::ReleaseGroupId(int groupId) {
	if (groupId < 0) {
		return;
	}
    

    // resize the group id vector if needed
	if (m_freeGroupTop >= m_freeGroupIds.size()) {
		m_freeGroupIds.resize(m_freeGroupTop + 1);
	}

    // add to free ids
	m_freeGroupIds[m_freeGroupTop] = groupId;
	m_freeGroupTop += 1;
}

void CrowdSimulationSystem::IncrementGroupCount(int groupId) {
	if (groupId < 0) {
		return;
	}
	if (m_groupCounts.size() <= groupId) {
		m_groupCounts.resize(groupId + 1, 0);
	}
	m_groupCounts[groupId] += 1;
}

void CrowdSimulationSystem::DecrementGroupCount(int groupId) {
	if (groupId < 0) {
		return;
	}
	if (groupId >= m_groupCounts.size()) {
		return;
	}
	if (m_groupCounts[groupId] > 0) {
		m_groupCounts[groupId] -= 1;
	}
}

void CrowdSimulationSystem::rebuildSpatialHash() {
	// clear the spatial hash so we can rebuild it
	for (auto& bucket : m_hashBuckets) {
		bucket.clear();
	}
	for (const Entity& entity : m_entityViews[0].dense()) {
		// put entity into the correct spatial bucket
		const TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
		const float posX = (transform.position.x - m_hashOrigin.x) / m_hashCellSize;
		const float posZ = (transform.position.z - m_hashOrigin.z) / m_hashCellSize;
		const int x = (std::min)((std::max)(static_cast<int>(posX), 0), m_hashWidth - 1);
		const int z = (std::min)((std::max)(static_cast<int>(posZ), 0), m_hashHeight - 1);
		m_hashBuckets[z * m_hashWidth + x].push_back(entity);
	}
}

bool CrowdSimulationSystem::wouldHitWall(const Vec3& position, float currentGroundY, float agentY) const {
	if (!m_flowCellLookupBuilt || m_flowWidth <= 0 || m_flowHeight <= 0 || m_flowCellSize <= 0.0f || m_flowCellLookup.empty()) {
		return false;
	}

	// Convert world position to cell coordinates to get the target cell
	const float posX = (position.x - m_flowOrigin.x) / m_flowCellSize;
	const float posZ = (position.z - m_flowOrigin.z) / m_flowCellSize;
	const int cellX = clampFloat(posX, 0.0f, m_flowWidth - 1);
	const int cellZ = clampFloat(posZ, 0.0f, m_flowHeight - 1);


	// Get the flow cell entity
	const Entity cellEntity = m_flowCellLookup[cellZ * m_flowWidth + cellX];
	if (cellEntity.id == UINT32_MAX || !Engine::HasComponent<FlowCellComponent>(cellEntity)) {
		return false;
	}

	// get the cell height
	const FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(cellEntity);

	if (cell.cellHeight <= 0) {
		return false;
	}
	
	// calculate the wall's height range
	// wall goes from y =0 to y =cellHeight * cellSize
	const float wallBottom = 0.0f;
	const float wallTop = cell.cellHeight * m_flowCellSize;
	const float wallHeight = wallTop - wallBottom;
	
	// Calculate the lower 3/4 of the wall
	const float lowerThreeQuartersTop = wallBottom + (wallHeight * 0.75f);
	
	// only count as collision if agent's y comp is within the lower 3/4 of the wall
	// this isy so that landing after hover is forgiving if you "just make it"
	const bool isInLowerThreeQuarters = agentY >= wallBottom && agentY <= lowerThreeQuartersTop;
	
	// Collision only if  agent is in the lower 3/4
	return isInLowerThreeQuarters;
}

void CrowdSimulationSystem::OnNotify(EventType type, const Event& payload) {
	if (type != EventType::DRONE_DETECTED && type != EventType::DRONE_CLEARED) {
		return;
	}
	const auto& detected = static_cast<const DroneDetected&>(payload);

	// add the cell to the pening pile of kill cells that need to be marked/cleared
	if (!m_flowCellLookupBuilt) {
		if (type == EventType::DRONE_DETECTED) {
			m_pendingKillCellPositions.push_back(detected.pos);
		} else {
			m_pendingClearCellPositions.push_back(detected.pos);
		}
		return;
	}

	// if the lookup has been built already we have to go update it ourselves
	// drone detect case
	if (type == EventType::DRONE_DETECTED) {
		const int idx = getFlowCellIndex(detected.pos);
		if (idx >= 0 && idx < m_flowCellLookup.size()) {
			const Entity killCellEntity = m_flowCellLookup[idx];
			if (killCellEntity.id != UINT32_MAX &&
			    Engine::HasComponent<FlowCellComponent>(killCellEntity)) {
				FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(killCellEntity);
				cell.isKill = true;
			}
		}
	} 
	
	// drone cleared case
	else {
		const int idx = getFlowCellIndex(detected.pos);
		if (idx >= 0 && idx < m_flowCellLookup.size()) {
			const Entity killCellEntity = m_flowCellLookup[idx];
			if (killCellEntity.id != UINT32_MAX &&
			    Engine::HasComponent<FlowCellComponent>(killCellEntity)) {
				FlowCellComponent& cell = Engine::GetComponent<FlowCellComponent>(killCellEntity);
				cell.isKill = false;
			}
		}
	}
}

int CrowdSimulationSystem::getFlowCellIndex(const Vec3& position) const {
	const float relX = (position.x - m_flowOrigin.x) / m_flowCellSize;
	const float relZ = (position.z - m_flowOrigin.z) / m_flowCellSize;
	const int x = clampFloat(relX, 0.0f, m_flowWidth - 1);
	const int z = clampFloat(relZ, 0.0f, m_flowHeight - 1);
	return z * m_flowWidth + x;
}

