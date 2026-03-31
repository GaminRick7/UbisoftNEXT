#pragma once

#include <vector>

#include "SVONode.h"
#include "Graph.h"
#include "engine/spatial/Octree.h"
#include "engine/components/ColliderComponent.h"
#include "engine/components/TransformComponent.h"

// sparse voxel octree that only subdivides when a collider overlaps a child node.
// currently it assumes colliders are inserted once upon scene load
// so no movement/removal handling and only support for static geometry
// -- will come back and add if time permits :0
class SparseVoxelOctree {
public:
	void Init(const OctreeConfig& config);
	void InsertCollider(const ColliderComponent& collider, const TransformComponent& transform);
	void Clear();

	//Todo remove
	std::vector<AABB> GetNodeBounds(bool blockedOnly = false) const;

	void GenerateNeighbours();
	bool GeneratePath(const Vec3& start, const Vec3& end);
	const std::vector<const SVONode*>& getPathList() const;

private:
	std::unique_ptr<SVONode> m_root;
	Graph m_graph;
	OctreeConfig m_config{};
	int maxDepth;

	void insertRecursive(SVONode* node, const AABB& bounds, int depth);
	void subdivide(SVONode* node);
	bool tryCollapse(SVONode* node);
	SVONode& findNearestNode(const Vec3& pos) const;
};
 