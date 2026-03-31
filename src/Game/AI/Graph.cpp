#include "Graph.h"
#include <unordered_map>

bool Graph::findPath(const SVONode& start, const SVONode& goal) {
	// let's implement A* \(-o-)/

	// let's create a temp node struct so we can use it in the max-heap
	struct Node
	{
		const SVONode* node;
		float gScore;
		float hScore;
	};

	// max-heap values are compared by the f scores (f = g + h)
	struct Compare
	{
		bool operator()(const Node& a, const Node& b) {
			return a.gScore + a.hScore > b.gScore + b.hScore;
		}
	};

	// clear old path
	pathList.clear();

	// if start == goal, path is just that node.
	if (start == goal) {
		pathList.push_back(&start);
		return true;
	}

	// we keep a "to check" max-heap ordered by best score
	std::priority_queue<Node, std::vector<Node>, Compare> openSet;
	openSet.push(Node{&start, 0,
		(start.bounds.getCenter() - goal.bounds.getCenter()).length()});

	//visited nodes are stores in closedSet
	std::unordered_set<const SVONode*> closedSet;

	// store the actual g scores of each node here,
	// updating it when a cheaper g score for that particular node is found

	// rationale: we want to always keep the cheapest g score to a node
	//				since that produces the lowest f-score for that node
	//				since the h-score (euclidean distance) remains unchanged
	std::unordered_map<const SVONode*, float> gScore;

	gScore[&start] = 0;

	// we shou
	std::unordered_map<const SVONode*, const SVONode*> cameFrom;

	while (!openSet.empty()) {

		// extract the max from the heap
		Node current = openSet.top();
		openSet.pop();

		// we reached the goal :D
		if (current.node == &goal) {
			pathList.clear();
			const SVONode* step = &goal;

			//lets trace back the path from the beginning
			while (step) {
				pathList.push_back(step);
				if (step == &start) break;
				auto it = cameFrom.find(step);
				if (it == cameFrom.end()) {
					return false; // no valid chain
				}
				step = it->second;
			}

			std::reverse(pathList.begin(), pathList.end());
			return true;
		}

		// if we found a cheaper route to that node already, this is stale so let's continue
		if (current.gScore > gScore[current.node]) continue;

		closedSet.insert(current.node);

		for (const auto* neighbour : current.node->neighbours) {
			// if we have already visited this node, continue
			if (closedSet.find(neighbour) != closedSet.end()) {
				continue;
			}

			// calculate the gScore of the node
			float newGScore = current.gScore +
				(current.node->bounds.getCenter() - neighbour->bounds.getCenter()).length();

			// only if this is the cheapest route to this node, do we add it to the open set
			if (gScore.find(neighbour) == gScore.end() || newGScore < gScore[neighbour]) {
				// update this node with the new values and re-route its source
				// to our current node to reflect the cheapest path
				gScore[neighbour] = newGScore;
				cameFrom[neighbour] = current.node;
				openSet.push(Node{neighbour, newGScore,
		(neighbour->bounds.getCenter() - goal.bounds.getCenter()).length()});
			}
		}
	}

	return false;
}