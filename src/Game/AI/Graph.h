#pragma once
#include <queue>
#include <vector>
#include "SVONode.h"

struct Graph {
	std::vector<const SVONode*> pathList;
	bool findPath(const SVONode& start, const SVONode& goal);
};
