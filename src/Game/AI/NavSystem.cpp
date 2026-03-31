#include "NavSystem.h"

#include <cmath>
#include <utility>
#include "engine/Engine.h"
#include "engine/components/ColliderComponent.h"
#include "engine/components/TransformComponent.h"

void NavSystem::Init() {
	m_octree.Clear();
	m_entityViews.clear();

	AABB worldBounds(Vec3(0.0f, 0, 0.0f), Vec3(100.0f, 100.0, 100.0f));
	OctreeConfig config;
	config.worldBounds = worldBounds;
	config.maxDepth = 5;
	m_octree.Init(config);

	//set mask to take in collider + transform
	ComponentMask mask;
	mask.set(Engine::GetTypeID<TransformComponent>());
	mask.set(Engine::GetTypeID<ColliderComponent>());

	addEntityView(mask);
}

void NavSystem::Shutdown() {
	m_octree.Clear();
}

std::vector<AABB> NavSystem::GetNodeBounds(bool blockedOnly) const {
	return m_octree.GetNodeBounds(blockedOnly);
}

void NavSystem::PopulateFromTrackedEntities() {
	if (m_entityViews.empty()) {
		return;
	}

	// interate through the  entity view with transform and collider components
	// and insert their colliders into the octree
	for (const Entity& entity : m_entityViews[0].dense()) {

		const ColliderComponent& collider = Engine::GetComponent<ColliderComponent>(entity);
		const TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
		m_octree.InsertCollider(collider, transform);
	}

	m_octree.GenerateNeighbours();
}

const std::vector<Vec3>& NavSystem::GetPath(const Vec3& start, const Vec3& end) {
	// generate a path
	m_octree.GeneratePath(start, end);
	const std::vector<const SVONode*>& path = m_octree.getPathList();
	m_pathPoints.clear();
	m_pathPoints.reserve(path.size());

	// save the node centers as path points
	for (const SVONode* node : path) {
		m_pathPoints.push_back(node->bounds.getCenter());
	}
	return m_pathPoints;
}

void NavSystem::clampCellFromPoint( const Vec3& point, int width, int length, const Vec3& origin, float cellSize, int& cellX, int& cellZ) const {
	const float localX = (point.x - origin.x) / cellSize;
	const float localZ = (point.z - origin.z) / cellSize;
	cellX = std::round(localX - 0.5f);
	cellZ = std::round(localZ - 0.5f);
	if (cellX < 0) {
		cellX = 0;
	} else if (cellX >= width) {
		cellX = width - 1;
	}
	if (cellZ < 0) {
		cellZ = 0;
	} else if (cellZ >= length) {
		cellZ = length - 1;
	}
}

bool NavSystem::isBlockedCell(const std::vector<int>& heightMap, int width, int cellX, int cellZ, float cellSize, float currentY, float hoverHeight) const {
	const int heightIndex = cellZ * width + cellX;
	const float cellTop = heightMap[heightIndex] * cellSize + hoverHeight;
	return cellTop > currentY + 0.001f;
}


void NavSystem::moveAxisToX(
	Vec3& current,
	float nextX,
	std::vector<Vec3>& path,
	const std::vector<int>& heightMap,
	int width,
	int length,
	const Vec3& origin,
	float cellSize,
	float hoverHeight
) const {
	Vec3 nextPoint(nextX, current.y, current.z);

	// get the cell height to where we wanna go
	int cellX = 0;
	int cellZ = 0;
	clampCellFromPoint(nextPoint, width, length, origin, cellSize, cellX, cellZ);
	const int heightIndex = cellZ * width + cellX;
	const float targetY = heightMap[heightIndex] * cellSize + hoverHeight;
	// if it is higher than us, we move on y first
	if (targetY > current.y + 0.001f) {
		current.y = targetY;
		path.push_back(current);
	}
	current.x = nextX;
	path.push_back(current);
	// if y is lower than us, we move on x first
	if (targetY < current.y - 0.001f) {
		current.y = targetY;
		path.push_back(current);
	}

	// the reasoning behind why we move on why first when going up and opposite when going down
	// is best visualized as a staircase- > if you are climbing stairs, you go up first and then forward second
}

void NavSystem::moveAxisToZ(
	Vec3& current,
	float nextZ,
	std::vector<Vec3>& path,
	const std::vector<int>& heightMap,
	int width,
	int length,
	const Vec3& origin,
	float cellSize,
	float hoverHeight
) const {
	Vec3 nextPoint(current.x, current.y, nextZ);

	// get the cell height to where we wanna go
	int cellX = 0;
	int cellZ = 0;
	clampCellFromPoint(nextPoint, width, length, origin, cellSize, cellX, cellZ);
	const int heightIndex = cellZ * width + cellX;
	const float targetY = heightMap[heightIndex] * cellSize + hoverHeight;
	// if it is higher than us, we move on z first
	if (targetY > current.y + 0.001f) {
		current.y = targetY;
		path.push_back(current);
	}
	current.z = nextZ;
	path.push_back(current);
	// if it is lower than us, we move on z first
	if (targetY < current.y - 0.001f) {
		current.y = targetY;
		path.push_back(current);
	}

	// the reasoning behind why we move on why first when going up and opposite when going down
	// is best visualized as a staircase- > if you are climbing stairs, you go up first and then forward second
}

std::vector<Vec3> NavSystem::BuildAxisAlignedPath(
	const std::vector<Vec3>& path,
	const std::vector<int>& heightMap,
	int width,
	int length,
	const Vec3& origin,
	float cellSize,
	float hoverHeight
) const {
	if (path.empty() || heightMap.empty() || width <= 0 || length <= 0 || cellSize <= 0.0f) {
		return path;
	}

	// the A* algo from the SVO builds a very precise path from A to B. This means that the drone
	// often barely squeezes by corners and moves diagonally. For a grid based game this is too unpredictable,
	// So let's ensure 2 things:
	//  - the drone's path points are the centers of grid cells
	//  - the drone can onky move on one axis at a time

	std::vector<Vec3> clampedPath;
	clampedPath.reserve(path.size());
	for (const Vec3& point : path) {
		int cellX = 0;
		int cellZ = 0;

		// let's get the cell's grid index and height from the position we are in
		clampCellFromPoint(point, width, length, origin, cellSize, cellX, cellZ);
		const int heightIndex = cellZ * width + cellX;
		const float cellHeight = heightMap[heightIndex] * cellSize;

		// we clamp the point to the center of a grid square/cube
		clampedPath.emplace_back(
			origin.x + (cellX + 0.5f) * cellSize,
			cellHeight + hoverHeight,
			origin.z + (cellZ + 0.5f) * cellSize
		);
	}

	if (clampedPath.size() <= 1) {
		return clampedPath;
	}

	// task 1: clamping to cell centers done!

	// since this is a grid based game, it would feel unfaor if the drone moved diagonally
	// so lets make sure the drone can only move on one axis at a time! :D
	std::vector<Vec3> axisPath;
	axisPath.reserve(clampedPath.size() * 2);
	axisPath.push_back(clampedPath.front());

	for (size_t i = 1; i < clampedPath.size(); ++i) {
		// keep track of the next and prev
		const Vec3& prev = axisPath.back();
		const Vec3& next = clampedPath[i];

		//check if the clamped path produces a change in x and z
		const bool changeX = std::fabs(prev.x - next.x) > 0.001f;
		const bool changeZ = std::fabs(prev.z - next.z) > 0.001f;
		Vec3 current = prev;

		// if both change, we have horizontal diagonal movement
		if (changeX && changeZ) {
			// build a directional vector that goes on the x axis first and then z axis second and vice versa
			Vec3 xThenZ(next.x, prev.y, prev.z);
			Vec3 zThenX(prev.x, prev.y, next.z);
			int xCellX = 0;
			int xCellZ = 0;
			int zCellX = 0;
			int zCellZ = 0;

			// get the cell's grid index when you take either path
			clampCellFromPoint(xThenZ, width, length, origin, cellSize, xCellX, xCellZ);
			clampCellFromPoint(zThenX, width, length, origin, cellSize, zCellX, zCellZ);

			// from the height map, determine if the cell that each path leads to is blocked
			const bool xBlocked = isBlockedCell(heightMap, width, xCellX, xCellZ, cellSize, current.y, hoverHeight);
			const bool zBlocked = isBlockedCell(heightMap, width, zCellX, zCellZ, cellSize, current.y, hoverHeight);

			// if x is not blcoked, lets default to that path
			if (!xBlocked) {
				//x first , z second
				moveAxisToX(current, next.x, axisPath, heightMap, width, length, origin, cellSize, hoverHeight);
				moveAxisToZ(current, next.z, axisPath, heightMap, width, length, origin, cellSize, hoverHeight);
			}
			// otherwise move on z
			else {
				// z first, x second
				moveAxisToZ(current, next.z, axisPath, heightMap, width, length, origin, cellSize, hoverHeight);
				moveAxisToX(current, next.x, axisPath, heightMap, width, length, origin, cellSize, hoverHeight);
			}
		} else {
			if (changeX) {
				moveAxisToX(current, next.x, axisPath, heightMap, width, length, origin, cellSize, hoverHeight);
			}
			if (changeZ) {
				moveAxisToZ(current, next.z, axisPath, heightMap, width, length, origin, cellSize, hoverHeight);
			}
		}

		axisPath.push_back(next);
	}

	return axisPath;
}