#pragma once
#include <unordered_set>
#include <memory>

#include "engine/spatial/AABB.h"

struct SVONode {
	SVONode()
		: id(s_nextId++) {}

	AABB bounds;
	std::unique_ptr<SVONode> children[8]; // 8 child nodes (null if not subdivided)
	bool isLeaf;
	bool isBlocked = false;
	uint32_t depth;
	uint64_t id;
	std::unordered_set<SVONode*> neighbours;

	bool operator==(const SVONode& other) const { return id == other.id; }

private:
	inline static int s_nextId = 0;
};
