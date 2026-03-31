#pragma once
#include "engine/ecs/core/System.h"
#include "math/vec3.h"
#include <vector>
#include <vector>

#include "engine/components/TransformComponent.h"
#include "engine/event/Observer.h"
#include "engine/event/Subject.h"

class CrowdAgentComponent;

class CrowdSimulationSystem : public System, public Observer, public Subject {
public:
	void Init();
	void rebuildSpatialHash();
	void Update(float deltaTime);
	void Shutdown();
	void SetMaxSpeed(float maxSpeed);

	void MarkGroupsDirty() { m_groupEntitiesBuilt = false; }
	int AcquireGroupId();
	void ReleaseGroupId(int groupId);
	const std::vector<int>& GetGroupCounts() const { return m_groupCounts; }
	void IncrementGroupCount(int groupId);
	void DecrementGroupCount(int groupId);

private:
	void OnNotify(EventType type, const Event& payload) override;

	Vec3 calculateSeparation(const Entity& entity, const TransformComponent& transform) const;
    Vec3 calculateGroupCohesion(const Entity& entity, const TransformComponent& transform) const;
	bool sampleFlowCell(const TransformComponent& transform, Vec3& outFlow, float& outGroundY, Vec3& outCellCenter, bool& outIsGoal) const;
	bool isWithinFlowStrip(const Vec3& agentPosition, const Vec3& cellCenter, const Vec3& incoming) const;
	void updateHoverMotion(const Vec3& newFlow, float groundY, float deltaSeconds, const Vec3& planarVelocity, const TransformComponent& transform, CrowdAgentComponent& agent, Vec3& outVelocity, Vec3& outPosition) const;
	void buildGroupEntities();
	void buildFlowCellLookup();
	bool wouldHitWall(const Vec3& position, float currentGroundY, float agentY) const;
	int getFlowCellIndex(const Vec3& position) const;

	float m_maxSpeed = 6.0f;
	float m_response = 2.0f;
	float m_gravity = -9.81f;
	float m_groundY = 0.6f;
	float m_hoverHeight = 6.0f;
	float m_hoverStrength = 8.0f;
	float m_hoverDamping = 1.0f;
	std::vector<Entity> m_flowCellLookup;
	bool m_flowCellLookupBuilt = false;
	int m_flowWidth = 0;
	int m_flowHeight = 0;
	float m_flowCellSize = 1.0f;
	Vec3 m_flowOrigin = Vec3(0.0f, 0.0f, 0.0f);
	float m_flowStripHalfWidthRatio = 0.25f;
	std::vector<Vec3> m_pendingKillCellPositions;
	std::vector<Vec3> m_pendingClearCellPositions;
	int m_goalReachedCount = 0;
	bool m_goalWinPrinted = false;

	// spatial hash for local neighbor queries
	float m_hashCellSize = 1.2f;
	int m_hashWidth = 0;
	int m_hashHeight = 0;
	Vec3 m_hashOrigin = Vec3(0.0f, 0.0f, 0.0f);
	std::vector<std::vector<Entity>> m_hashBuckets;

	float m_separationRadius = 0.5f;
	float m_separationStrength = 1.1f;

	float m_cohesionRadius = 6.0f;
	float m_cohesionStrength = 0.0f;
	std::vector<Entity> m_groupEntities;
	bool m_groupEntitiesBuilt = false;
	std::vector<int> m_freeGroupIds;
	size_t m_freeGroupTop = 0;
	int m_nextGroupId = 0;
	std::vector<int> m_groupCounts;
};
