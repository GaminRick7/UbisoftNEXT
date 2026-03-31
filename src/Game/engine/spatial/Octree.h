#pragma once
#include "OctreeNode.h"
#include "AABB.h"
#include "Ray.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <cstdint>

#include "Frustum.h"


struct OctreeConfig {
    AABB worldBounds;
    int maxDepth = 8;
    int maxEntitiesPerNode = 8;
    float minNodeSize = 1.0f;
};

class Octree {
public:
    Octree();
    void Init(const OctreeConfig& config);
    void Clear();

    void Insert(Entity e, const AABB& bounds);
    void Remove(Entity e);

    // update entity bounds
    void Update(Entity e, const AABB& newBounds);
    
    // query entities within AABB
    std::vector<Entity> QueryAABB(const AABB& bounds) const;

    // query entities with Frustum
    std::vector<Entity> QueryFrustum(const Frustum& frustum) const;

    std::vector<Entity> QuerySphere(const Sphere& sphere) const;
	std::vector<std::pair<Entity, float>> QueryRay(const Ray& ray) const;
    
    // DEBUG: get all node bounds for visualization
    std::vector<AABB> GetNodeBounds() const;
protected:
    std::unique_ptr<OctreeNode> m_root;
    AABB m_worldBounds;
    OctreeConfig m_config{};

    std::unordered_map<EntityID, AABB> m_entityBounds;
    
    // helper methods
    void subdivide(OctreeNode* node);
    void insertRecursive(OctreeNode* node, Entity e, const AABB& bounds, int depth);
    void removeRecursive(OctreeNode* node, Entity e);
    void queryAABBRecursive(const OctreeNode* node, const AABB& queryBounds, std::vector<Entity>& results) const;
    void queryFrustumRecursive(const OctreeNode* node, const Frustum& frustum, std::vector<Entity>& results) const;
    void querySphereRecursive(const OctreeNode* node, const Sphere& sphere, std::vector<Entity>& results) const;
	void queryRayRecursive(const OctreeNode* node, const Ray& ray, std::vector<std::pair<Entity, float>>& results) const;
    void getNodeBoundsRecursive(const OctreeNode* node, std::vector<AABB>& outBounds) const;
    
    // calculate child AABB for given octant index (0-7) it is used for subidivision
    AABB calculateChildBounds(const AABB& parentBounds, int octantIndex) const;

    bool boundsOverlap(const AABB& a, const AABB& b) const;

    bool shouldSubdivide(const OctreeNode* node) const;
};

