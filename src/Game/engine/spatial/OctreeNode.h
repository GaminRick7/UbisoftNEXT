#pragma once
#include "AABB.h"
#include <vector>
#include <memory>
#include "engine/ecs/core/Entity.h"

class Octree;

// Octree node structure
// each node represents a 3D region of space (AABB)
// leaf nodes contain entities, internal nodes have 8 children
struct OctreeNode {
    AABB bounds;
    std::vector<Entity> entities; 
    std::unique_ptr<OctreeNode> children[8]; // 8 child nodes (null if not subdivided)
    bool isLeaf;
    uint32_t depth; // depth in tree (0 = root)

    OctreeNode() : isLeaf(true), depth(0) {}

    bool hasChildren() const {
        return children[0] != nullptr;
    }
    
    int getChildCount() const {
        int count = 0;
        for (int i = 0; i < 8; i++) {
            if (children[i] != nullptr) {
                count++;
            }
        }
        return count;
    }
};

