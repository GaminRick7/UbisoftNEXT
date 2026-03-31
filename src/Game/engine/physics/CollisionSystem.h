#pragma once
#include "engine/ecs/core/System.h"
#include "engine/components/TransformComponent.h"
#include "engine/components/RigidbodyComponent.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/ColliderComponent.h"
#include "engine/spatial/AABB.h"
#include <vector>

#include "engine/event/Subject.h"

// All collisions are detected by the collision system 
// and then propagated to any where else that needs teh collision data via events
class CollisionSystem : public System, public Subject {
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();
	ColliderComponent ComputeWorldCollider(Entity entity) const;

private:
	void BroadPhase(float deltaTime);
	void NarrowPhase(float deltaTime);
	AABB ComputeWorldBounds(Entity entity) const;

	std::vector<std::pair<Entity, Entity>> m_candidatePairs;
};


