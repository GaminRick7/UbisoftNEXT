#include "Octree.h"
#include "math/vec3.h"
#include <algorithm>
#include <cmath>
#include <limits>

#include "Sphere.h"

// this is a strict single-ownership octree: entities live in the deepest node whose tight AABB fully contains them,
// nodes subdivide only when they can pass at least one entity down to a child, and queries/removals always inspect
// the node's own entities plus its children sbecause both can contain entities (they are mixed)

Octree::Octree() {
    m_root = nullptr;
}

void Octree::Init(const OctreeConfig& config) {
    m_config = config;
    m_worldBounds = config.worldBounds;
    m_root = std::make_unique<OctreeNode>();
    m_root->bounds = config.worldBounds;
    m_root->isLeaf = true;
    m_root->depth = 0;
    m_entityBounds.clear();
}

void Octree::Clear() {
    m_root.reset();
    m_entityBounds.clear();
}

void Octree::Insert(Entity e, const AABB& bounds) {
    if (!m_root) {
        return;
    }
    
    // store entity bounds for later removal
    m_entityBounds[e.id] = bounds;
    
    insertRecursive(m_root.get(), e, bounds, 0);
}

void Octree::insertRecursive(OctreeNode* node, Entity e, const AABB& bounds, int depth) {
    if (!boundsOverlap(bounds, node->bounds)) {
        return;
    }

    // keep pushing the entity down until a child fully owns it (like a child node fully ocntainsd it)
    // otherwise it stays with this node
    if (node->isLeaf) {
        node->entities.push_back(e);

        Vec3 nodeSize = node->bounds.getSize();
        float minSize = (std::min)({nodeSize.x, nodeSize.y, nodeSize.z});
        const bool canSubdivide = depth < m_config.maxDepth && minSize > m_config.minNodeSize;

        if (canSubdivide && node->entities.size() > static_cast<size_t>(m_config.maxEntitiesPerNode) &&
            shouldSubdivide(node)) {
            subdivide(node);
        }
        return;
    }

    // internal node so try to descend into a child that fully contains the bounds
    for (uint32_t i = 0; i < 8; i++) {
        if (node->children[i] && node->children[i]->bounds.contains(bounds)) {
            insertRecursive(node->children[i].get(), e, bounds, depth + 1);
            return;
        }
    }

    // no child fully contains the entity, keep it in this node
    node->entities.push_back(e);
}

void Octree::subdivide(OctreeNode* node) {
    // create 8 child nodes
    for (int i = 0; i < 8; i++) {
        node->children[i] = std::make_unique<OctreeNode>();
        node->children[i]->bounds = node->bounds.calculateChildBounds(i);
        node->children[i]->isLeaf = true;
        node->children[i]->depth = node->depth + 1;
    }
    
    node->isLeaf = false;

    std::vector<Entity> remainingEntities;
    remainingEntities.reserve(node->entities.size());

    for (Entity& e : node->entities) {
        const AABB entityBounds = m_entityBounds[e.id];
        bool movedToChild = false;

        for (int i = 0; i < 8; i++) {
            if (node->children[i]->bounds.contains(entityBounds)) {
                insertRecursive(node->children[i].get(), e, entityBounds, node->children[i]->depth);
                movedToChild = true;
                break;
            }
        }

        if (!movedToChild) {
            remainingEntities.push_back(e);
        }
    }

    node->entities.swap(remainingEntities);
}

bool Octree::boundsOverlap(const AABB& a, const AABB& b) const {
    return a.intersects(b);
}

bool Octree::shouldSubdivide(const OctreeNode* node) const {
    for (const Entity& e : node->entities) {
        const AABB& entityBounds = m_entityBounds.at(e.id);
        for (int i = 0; i < 8; i++) {
            AABB childBounds = node->bounds.calculateChildBounds(i);
            if (childBounds.contains(entityBounds)) {
                return true;
            }
        }
    }
    return false;
}


void Octree::Remove(Entity e) {
    if (!m_root) {
        return;
    }
    
    // remove from tree
    removeRecursive(m_root.get(), e);
    
    // remove from bounds map
    m_entityBounds.erase(e.id);
}

void Octree::removeRecursive(OctreeNode* node, Entity e) {
    auto it = std::find(node->entities.begin(), node->entities.end(), e);
    if (it != node->entities.end()) {
        node->entities.erase(it);
        return;
    }

    if (!node->isLeaf) {
        AABB entityBounds = m_entityBounds[e.id];
        for (int i = 0; i < 8; i++) {
            if (node->children[i] && boundsOverlap(entityBounds, node->children[i]->bounds)) {
                removeRecursive(node->children[i].get(), e);
            }
        }
    }
}

void Octree::Update(Entity e, const AABB& newBounds) {
    Remove(e);
    Insert(e, newBounds);
}

std::vector<Entity> Octree::QueryAABB(const AABB& bounds) const {
    std::vector<Entity> results;
    
    if (!m_root) {
        return results;
    }
    
    queryAABBRecursive(m_root.get(), bounds, results);
    return results;
}

void Octree::queryAABBRecursive(const OctreeNode* node, const AABB& queryBounds, std::vector<Entity>& results) const {
    if (!boundsOverlap(queryBounds, node->bounds)) {
        return;
    }
    
    for (const Entity& e : node->entities) {
        const AABB& entityBounds = m_entityBounds.at(e.id);
        if (boundsOverlap(queryBounds, entityBounds)) {
            results.push_back(e);
        }
    }
    
    if (!node->isLeaf) {
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                queryAABBRecursive(node->children[i].get(), queryBounds, results);
            }
        }
    }
}

std::vector<Entity> Octree::QueryFrustum(const Frustum& frustum) const {
    std::vector<Entity> results;

    if (!m_root) {
        return results;
    }

    queryFrustumRecursive(m_root.get(), frustum, results);

    return results;
}

void Octree::queryFrustumRecursive(const OctreeNode* node, const Frustum& frustum, std::vector<Entity>& results) const {
    if (!frustum.intersects(node->bounds)) {
        return;
    }

    for (const Entity& e: node->entities) {
        const AABB& entityBounds = m_entityBounds.at(e.id);
        if (frustum.intersects(entityBounds)) {
            results.push_back(e);
        }
    }

    if (!node->isLeaf) {
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                queryFrustumRecursive(node->children[i].get(), frustum, results);
            }
        }
    }
}

std::vector<Entity> Octree::QuerySphere(const Sphere& sphere) const {
    std::vector<Entity> results;

    if (!m_root) {
        return results;
    }

    querySphereRecursive(m_root.get(), sphere, results);
    return results;
}

std::vector<std::pair<Entity, float>> Octree::QueryRay(const Ray& ray) const {
	std::vector<std::pair<Entity, float>> results;

	if (!m_root) {
		return results;
	}

	queryRayRecursive(m_root.get(), ray, results);

    //return all the hit entities and their distance as a pair in order of closest to farthest
	std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) {
		return a.second < b.second;
	});
	return results;
}

void Octree::queryRayRecursive(const OctreeNode* node, const Ray& ray, std::vector<std::pair<Entity, float>>& results) const {
	float nodeHit;
	if (!node->bounds.intersects(ray, nodeHit)) {
		return;
	}

	for (const Entity& e : node->entities) {
		const AABB& entityBounds = m_entityBounds.at(e.id);
		float hitT;
		if (entityBounds.intersects(ray, hitT)) {
			results.emplace_back(e, hitT);
		}
	}

	if (!node->isLeaf) {
		for (int i = 0; i < 8; i++) {
			if (node->children[i]) {
				queryRayRecursive(node->children[i].get(), ray, results);
			}
		}
	}
}

void Octree::querySphereRecursive(const OctreeNode* node, const Sphere& sphere, std::vector<Entity>& results) const {
    if (!node->bounds.intersects(sphere)) {
        return;
    }

    for (const Entity& e : node->entities) {
        const AABB& entityBounds = m_entityBounds.at(e.id);
        if (entityBounds.intersects(sphere)) {
            results.push_back(e);
        }
    }

    if (!node->isLeaf) {
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                querySphereRecursive(node->children[i].get(), sphere, results);
            }
        }
    }
}


std::vector<AABB> Octree::GetNodeBounds() const {
    std::vector<AABB> nodeBounds;
    if (m_root) {
        getNodeBoundsRecursive(m_root.get(), nodeBounds);
    }
    return nodeBounds;
}

void Octree::getNodeBoundsRecursive(const OctreeNode* node, std::vector<AABB>& outBounds) const {
    if (!node) {
        return;
    }
    
    if (!node->isLeaf) {
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                getNodeBoundsRecursive(node->children[i].get(), outBounds);
            }
        }
    } else {
        outBounds.push_back(node->bounds);
    }
}

