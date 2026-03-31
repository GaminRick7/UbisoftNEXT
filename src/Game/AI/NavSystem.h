#pragma once
#include "SparseVoxelOctree.h"
#include "engine/ecs/core/System.h"
#include "math/vec3.h"
#include <vector>

class NavSystem : public System {
public:
	void Init();
	void Update();
	void Shutdown();

	std::vector<AABB> GetNodeBounds(bool blockedOnly) const;
	void PopulateFromTrackedEntities();
	const std::vector<Vec3>& GetPath(const Vec3& start, const Vec3& end);
	std::vector<Vec3> BuildAxisAlignedPath(const std::vector<Vec3>& path, const std::vector<int>& heightMap, int width, int length, const Vec3& origin, float cellSize, float hoverHeight) const;
private:
	void clampCellFromPoint(const Vec3& point, int width, int length, const Vec3& origin, float cellSize, int& cellX, int& cellZ) const;
	bool isBlockedCell(const std::vector<int>& heightMap, int width, int cellX, int cellZ, float cellSize, float currentY, float hoverHeight) const;
	void moveAxisToX(Vec3& current, float nextX, std::vector<Vec3>& path, const std::vector<int>& heightMap, int width, int length, const Vec3& origin, float cellSize, float hoverHeight) const;
	void moveAxisToZ(Vec3& current, float nextZ, std::vector<Vec3>& path, const std::vector<int>& heightMap, int width, int length, const Vec3& origin, float cellSize, float hoverHeight) const;
	SparseVoxelOctree m_octree;

	// vector is updated whenever a new path is requested
	std::vector<Vec3> m_pathPoints;
};
