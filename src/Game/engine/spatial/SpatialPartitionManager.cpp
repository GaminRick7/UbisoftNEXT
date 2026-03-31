#include "SpatialPartitionManager.h"

#include "engine/Engine.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/TransformComponent.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "math/mat4.h"
#include <limits>

void SpatialPartitionManager::Init() {
	m_mask.set(Engine::GetTypeID<TransformComponent>());
	m_mask.set(Engine::GetTypeID<BoundsComponent>());

	AABB worldBounds(Vec3(0.0f, 0.0f, 0.0f), Vec3(100.0f, 100.0f, 100.0f));
	OctreeConfig config;
	config.worldBounds = worldBounds;
	m_octree.Init(config);
}

void SpatialPartitionManager::Update(float deltaTime) {
	detectEntityMovement();
	updateDirtyEntities();
}

void SpatialPartitionManager::Shutdown() {
	m_octree.Clear();
	m_entityPositions.clear();
	m_dirtyEntities.clear();
}

std::vector<Entity> SpatialPartitionManager::QueryFrustum(const Frustum& frustum) {
	return m_octree.QueryFrustum(frustum);
}

std::vector<Entity> SpatialPartitionManager::QueryAABB(const AABB& bounds) const {
	return m_octree.QueryAABB(bounds);
}

std::vector<Entity> SpatialPartitionManager::QuerySphere(const Sphere& sphere) const {
	return m_octree.QuerySphere(sphere);
}

std::vector<std::pair<Entity, float>> SpatialPartitionManager::QueryRay(const Ray& ray) const {
	return m_octree.QueryRay(ray);
}

std::vector<AABB> SpatialPartitionManager::GetNodeBounds() const {
	return m_octree.GetNodeBounds();
}

AABB SpatialPartitionManager::computeWorldBounds(Entity e) const {
	// get local bounds from BoundsComponent
	const BoundsComponent& boundsComp = Engine::GetComponent<BoundsComponent>(e);
	const AABB& localBounds = boundsComp.bounds;
	
	// get transform
	const TransformComponent& transform = Engine::GetComponent<TransformComponent>(e);
	
	// build transform matrix
	Mat4 transformMatrix = Mat4::translation(transform.position) * Mat4::rotation(transform.rotation)
	                      * Mat4::scaling(transform.scale);
	
	// get local AABB center and extents
	Vec3 center = localBounds.getCenter();
	Vec3 extents = localBounds.getExtents();
	
	// transform 8 corners of local AABB to world space
	Vec3 corners[8] = {
		center + Vec3(-extents.x, -extents.y, -extents.z),
		center + Vec3( extents.x, -extents.y, -extents.z),
		center + Vec3(-extents.x,  extents.y, -extents.z),
		center + Vec3( extents.x,  extents.y, -extents.z),
		center + Vec3(-extents.x, -extents.y,  extents.z),
		center + Vec3( extents.x, -extents.y,  extents.z),
		center + Vec3(-extents.x,  extents.y,  extents.z),
		center + Vec3( extents.x,  extents.y,  extents.z),
	};
	
	// Find min/max of transformed corners
	Vec3 worldMin(
		(std::numeric_limits<float>::max)(),
		(std::numeric_limits<float>::max)(),
		(std::numeric_limits<float>::max)()
	);

	Vec3 worldMax(
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest()
	);
	
	for (const Vec3& corner : corners) {
		Vec4 transformed = transformMatrix * Vec4(corner.x, corner.y, corner.z, 1.0f);
		worldMin.x = (std::min)(worldMin.x, transformed.x);
		worldMin.y = (std::min)(worldMin.y, transformed.y);
		worldMin.z = (std::min)(worldMin.z, transformed.z);
		worldMax.x = (std::max)(worldMax.x, transformed.x);
		worldMax.y = (std::max)(worldMax.y, transformed.y);
		worldMax.z = (std::max)(worldMax.z, transformed.z);
	}
	
	return AABB(worldMin, worldMax);
}

void SpatialPartitionManager::detectEntityMovement() {
	for (const auto& [entity, prevPosition] : m_entityPositions) {
		const TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
		if (transform.position.x != prevPosition.x || 
		    transform.position.y != prevPosition.y || 
		    transform.position.z != prevPosition.z) {
			m_dirtyEntities.push_back(entity);
		}
	}
}

void SpatialPartitionManager::updateDirtyEntities() {
	for (Entity e : m_dirtyEntities) {
		// Compute new world bounds
		AABB worldBounds = computeWorldBounds(e);
		
		// Update in octree
		m_octree.Update(e, worldBounds);
		
		// Update stored position
		const TransformComponent& transform = Engine::GetComponent<TransformComponent>(e);
		m_entityPositions[e] = transform.position;
	}
	
	// Clear dirty list
	m_dirtyEntities.clear();
}

void SpatialPartitionManager::EntityMaskChanged(Entity e, ComponentMask mask) {
	const bool hasRequiredComponents = (m_mask & mask) == m_mask;
	const bool alreadyTracked = m_entityPositions.find(e) != m_entityPositions.end();

	if (hasRequiredComponents) {
		AABB worldBounds = computeWorldBounds(e);

		if (!alreadyTracked) {
			m_octree.Insert(e, worldBounds);
		}

		const TransformComponent& transform = Engine::GetComponent<TransformComponent>(e);
		m_entityPositions[e] = transform.position;
	} else if (alreadyTracked) {
		m_octree.Remove(e);
		m_entityPositions.erase(e);
	}
}

void SpatialPartitionManager::EntityDestroyed(Entity e, ComponentMask mask) {
	if ((m_mask & mask) == m_mask) {
		// Remove from octree and tracking
		m_octree.Remove(e);
		m_entityPositions.erase(e);
	}
}