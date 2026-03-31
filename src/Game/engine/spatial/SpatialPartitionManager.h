#pragma once
#include "Octree.h"
#include "engine/ecs/core/constants.h"
#include "engine/ecs/core/System.h"
#include "AABB.h"
#include "Sphere.h"
#include "Ray.h"
#include <unordered_map>
#include <vector>

class SpatialPartitionManager {
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();

	std::vector<Entity> QueryFrustum(const Frustum& frustum);
	std::vector<Entity> QueryAABB(const AABB& bounds) const;
	std::vector<Entity> QuerySphere(const Sphere& sphere) const;
	std::vector<std::pair<Entity, float>> QueryRay(const Ray& ray) const;
	std::vector<AABB> GetNodeBounds() const;

	void EntityMaskChanged(Entity e, ComponentMask mask);
	void EntityDestroyed(Entity e, ComponentMask mask);

private:
	Octree m_octree;
	std::unordered_map<Entity, Vec3> m_entityPositions;
	std::vector<Entity> m_dirtyEntities;
	ComponentMask m_mask;

	AABB computeWorldBounds(Entity e) const;
	void updateDirtyEntities(); // Update only dirty entities in octree
	void detectEntityMovement(); // Compare current vs previous positions, mark as dirty
};
