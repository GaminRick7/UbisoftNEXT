#include "SparseVoxelOctree.h"

#include "engine/Engine.h"

#include <cmath>
#include <queue>
#include <unordered_set>

void SparseVoxelOctree::Init(const OctreeConfig& config) {
	m_config = config;
	maxDepth = config.maxDepth;

	m_root = std::make_unique<SVONode>();
	m_root->bounds = config.worldBounds;
	m_root->isLeaf = true;
	m_root->isBlocked = false;
	m_root->depth = 0;
}

void SparseVoxelOctree::Clear() {
	m_root.reset();
}

void SparseVoxelOctree::InsertCollider(const ColliderComponent& collider, const TransformComponent& transform) {
	AABB worldBounds = collider.computeWorldBounds(transform.position, transform.rotation, transform.scale);
	insertRecursive(m_root.get(), worldBounds, 0);
}

std::vector<AABB> SparseVoxelOctree::GetNodeBounds(bool blockedOnly) const {
	std::vector<AABB> bounds;

	if (!m_root) {
		return bounds;
	}

	std::vector<const SVONode*> stack;
	stack.push_back(m_root.get());

	while (!stack.empty()) {
		const SVONode* node = stack.back();
		stack.pop_back();

		if (!blockedOnly || node->isBlocked) {
			bounds.push_back(node->bounds);
		}

		if (!node->isLeaf) {
			for (const auto& child : node->children) {
				if (child) {
					stack.push_back(child.get());
				}
			}
		}
	}

	return bounds;
}

void SparseVoxelOctree::insertRecursive(SVONode* node, const AABB& bounds, int depth) {
	// 1. Skip if the collider doesn't overlap the node bounds
	if (!node->bounds.intersects(bounds)) {
		return;
	}

	if (node->isLeaf && node->isBlocked) {
		return;
	}

	if (node->isLeaf) {
		const Vec3 nodeSize = node->bounds.getSize();
		const bool aboveMinSize =
			nodeSize.x > m_config.minNodeSize &&
			nodeSize.y > m_config.minNodeSize &&
			nodeSize.z > m_config.minNodeSize;
		const bool canSubdivide = depth < maxDepth && aboveMinSize;

		if (!canSubdivide) {
			node->isBlocked = true;
			return;
		}

		subdivide(node);
	}

	for (auto& child : node->children) {
		if (child && child->bounds.intersects(bounds) && !child->isBlocked) {
			insertRecursive(child.get(), bounds, depth + 1);
		}
	}

	tryCollapse(node);
}

void SparseVoxelOctree::subdivide(SVONode* node) {
	node->isLeaf = false;
	node->isBlocked = false; // blocked is invalid for internal nodes

	for (int i = 0; i < 8; i++) {
		node->children[i] = std::make_unique<SVONode>();
		node->children[i]->bounds = node->bounds.calculateChildBounds(i);
		node->children[i]->isLeaf = true;
		node->children[i]->isBlocked = false;
		node->children[i]->depth = node->depth + 1;
	}
}

bool SparseVoxelOctree::tryCollapse(SVONode* node) {
	if (node->isLeaf) return false;

	for (auto& child : node->children) {
		if (!child || !child->isLeaf || !child->isBlocked) {
			return false;
		}
	}

	// collapse
	for (auto& child : node->children) {
		child.reset();
	}

	node->isLeaf = true;
	node->isBlocked = true;
	return true;
}


void SparseVoxelOctree::GenerateNeighbours() {
	std::vector<SVONode*> nodes;

	if (!m_root) {
		return;
	}

	m_graph.pathList.clear();

	std::vector<SVONode*> stack;
	stack.push_back(m_root.get());

	while (!stack.empty()) {
		SVONode* node = stack.back();
		stack.pop_back();

		if (!node->isLeaf) {
			for (auto& child : node->children) {
				if (child) {
					stack.push_back(child.get());
				}
			}
		} else if (!node->isBlocked) {
			nodes.push_back(node);
		}
	}

	for (SVONode* node : nodes) {
		node->neighbours.clear();
	}

	for (size_t i = 0; i < nodes.size(); i++) {
		for (size_t j = i + 1; j < nodes.size(); j++) {
			if (nodes.at(i)->bounds.adjacent(nodes.at(j)->bounds)) {
				SVONode* nodeA = nodes[i];
				SVONode* nodeB = nodes[j];
				nodeA->neighbours.insert(nodeB);
				nodeB->neighbours.insert(nodeA);
			}
		}
	}
}

SVONode& SparseVoxelOctree::findNearestNode(const Vec3& pos) const {
	// Search all leaves for the closest unblocked node.
	SVONode* closestUnblocked = nullptr;
	float bestDistSq = (std::numeric_limits<float>::max)();
	SVONode* fallbackLeaf = nullptr;
	std::vector<SVONode*> stack;
	stack.push_back(m_root.get());

	while (!stack.empty()) {
		SVONode* current = stack.back();
		stack.pop_back();

		if (!current) {
			continue;
		}

		if (!current->isLeaf) {
			for (auto& child : current->children) {
				if (child) {
					stack.push_back(child.get());
				}
			}
			continue;
		}

		if (!fallbackLeaf) {
			fallbackLeaf = current;
		}

		if (current->isBlocked) {
			continue;
		}

		const Vec3 center = current->bounds.getCenter();
		const Vec3 diff = center - pos;
		const float distSq = diff.dot(diff);
		if (distSq < bestDistSq) {
			bestDistSq = distSq;
			closestUnblocked = current;
		}
	}

	if (closestUnblocked) {
		return *closestUnblocked;
	}
	if (fallbackLeaf) {
		return *fallbackLeaf;
	}
	return *m_root;
}

const std::vector<const SVONode*>& SparseVoxelOctree::getPathList() const {
	return m_graph.pathList;
}

bool SparseVoxelOctree::GeneratePath(const Vec3& start, const Vec3& end) {
	if (!m_root) {
		return false;
	}
	return m_graph.findPath(findNearestNode(start), findNearestNode(end));
}