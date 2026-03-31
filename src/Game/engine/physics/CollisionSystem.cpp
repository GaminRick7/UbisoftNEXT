#include "CollisionSystem.h"
#include "engine/Engine.h"
#include "math/mat4.h"
#include <cmath>
#include <unordered_set>

#include "CollisionEvent.h"


struct ContactInfo {
	Vec3 normal = Vec3(0.0f, 1.0f, 0.0f);
	float penetration = 0.0f;
};

Vec3 ClosestPointOnAABB(const AABB& box, const Vec3& point) {
	// find the closest point on AABB to point
	return Vec3(
		(std::max)(box.minCorner.x, (std::min)(point.x, box.maxCorner.x)),
		(std::max)(box.minCorner.y, (std::min)(point.y, box.maxCorner.y)),
		(std::max)(box.minCorner.z, (std::min)(point.z, box.maxCorner.z))
	);
}

bool ComputeSphereSphereContact(const Sphere& first, const Sphere& second, ContactInfo& out) {
	// get the distance squared between spheres
	Vec3 delta = second.center - first.center;
	const float distSq = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

	// if the distance is almost 0, resort to default upward normal (0,1,0) and set penetration
	// to the sum of both radii
	if (distSq < 1e-12f) {
		out.normal = Vec3(0.0f, 1.0f, 0.0f);
		out.penetration = first.radius + second.radius;
		return true;
	}

	// otherwise claculate penetration and store result in out
	const float dist = std::sqrt(distSq);
	const float penetration = (first.radius + second.radius) - dist;
	if (penetration <= 0.0f) {
		return false;
	}

	out.normal = delta.normalize();
	out.penetration = penetration;
	return true;
}

bool ComputeSphereAABBContact(const Sphere& sphere, const AABB& box, bool sphereIsFirst, ContactInfo& out) {
	// find closes point on AABB to the center of sphere
	const Vec3 closest = ClosestPointOnAABB(box, sphere.center);
	Vec3 dir = closest - sphere.center; // from sphere to box
	const float distSq = dir.x * dir.x + dir.y * dir.y + dir.z * dir.z;

	// if the distance is greater than almost 0
	if (distSq > 1e-12f) {
		const float dist = std::sqrt(distSq);
		const float penetration = sphere.radius - dist;
		if (penetration <= 0.0f) {
			return false;
		}

		Vec3 normal = dir * (1.0f / dist);
		// flip the normal if our A = box and B = sphere
		out.normal = sphereIsFirst ? normal : (normal * -1.0f);
		out.penetration = penetration;
		return true;
	}

	// sphere center is inside the box, pick the closest face
	// rationale: min and max, despite being corners, can represent the six faces
	//				because each face is a plane where one coordinate is fixed to either min or max
	const float distToMinX = sphere.center.x - box.minCorner.x;
	const float distToMaxX = box.maxCorner.x - sphere.center.x;
	const float distToMinY = sphere.center.y - box.minCorner.y;
	const float distToMaxY = box.maxCorner.y - sphere.center.y;
	const float distToMinZ = sphere.center.z - box.minCorner.z;
	const float distToMaxZ = box.maxCorner.z - sphere.center.z;

	float minDist = distToMinX;
	Vec3 normal(-1.0f, 0.0f, 0.0f);

	if (distToMaxX < minDist) {
		minDist = distToMaxX;
		normal = Vec3(1.0f, 0.0f, 0.0f);
	}
	if (distToMinY < minDist) {
		minDist = distToMinY;
		normal = Vec3(0.0f, -1.0f, 0.0f);
	}
	if (distToMaxY < minDist) {
		minDist = distToMaxY;
		normal = Vec3(0.0f, 1.0f, 0.0f);
	}
	if (distToMinZ < minDist) {
		minDist = distToMinZ;
		normal = Vec3(0.0f, 0.0f, -1.0f);
	}
	if (distToMaxZ < minDist) {
		minDist = distToMaxZ;
		normal = Vec3(0.0f, 0.0f, 1.0f);
	}

	const float penetration = sphere.radius - minDist;
	if (penetration <= 0.0f) {
		return false;
	}

	// flip the normal if our A = box and B = sphere
	out.normal = sphereIsFirst ? normal : (normal * -1.0f);
	out.penetration = penetration;
	return true;
}

bool ComputeAABBContact(const AABB& first, const AABB& second, ContactInfo& out) {
	const Vec3 centerFirst = first.getCenter();
	const Vec3 centerSecond = second.getCenter();
	const Vec3 extentsFirst = first.getExtents();
	const Vec3 extentsSecond = second.getExtents();

	// boxes overlap if the delta is less than extent + extent
	const Vec3 delta = centerSecond - centerFirst;

	const float overlapX = (extentsFirst.x + extentsSecond.x) - std::abs(delta.x);
	const float overlapY = (extentsFirst.y + extentsSecond.y) - std::abs(delta.y);
	const float overlapZ = (extentsFirst.z + extentsSecond.z) - std::abs(delta.z);

	if (overlapX <= 0.0f || overlapY <= 0.0f || overlapZ <= 0.0f) {
		return false;
	}

	// find the axis with the minimal overlap
	// we can then move along that axis to de-penetrate with the least displacement

	// x has the greatest overlap
	if (overlapX <= overlapY && overlapX <= overlapZ) {
		out.penetration = overlapX;
		out.normal = Vec3((delta.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f);
	}
	// y has the greatest overlap
	else if (overlapY <= overlapZ) {
		out.penetration = overlapY;
		out.normal = Vec3(0.0f, (delta.y >= 0.0f) ? 1.0f : -1.0f, 0.0f);
	}
	// z has the greatest overlap
	else {
		out.penetration = overlapZ;
		out.normal = Vec3(0.0f, 0.0f, (delta.z >= 0.0f) ? 1.0f : -1.0f);
	}

	return true;
}

bool BuildContactInfo(const ColliderComponent& first, const ColliderComponent& second, ContactInfo& out) {
	// cases representing collision between spheres and AABBs
	if (first.isSphere()) {
		if (second.isSphere()) {
			return ComputeSphereSphereContact(first.sphere, second.sphere, out);
		}
		return ComputeSphereAABBContact(first.sphere, second.box, true, out);
	}

	if (second.isSphere()) {
		return ComputeSphereAABBContact(second.sphere, first.box, false, out);
	}

	return ComputeAABBContact(first.box, second.box, out);
}


// hash two entities together so we can toss the pair into an unordered_set :)
struct EntityPairHash {
	size_t operator()(const std::pair<Entity, Entity>& value) const {
        // entity already has its own hash function so let's use that and some more factors!
		size_t seed = std::hash<Entity>{}(value.first);
		seed ^= std::hash<Entity>{}(value.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};

struct EntityPairEqual {
	bool operator()(const std::pair<Entity, Entity>& a, const std::pair<Entity, Entity>& b) const {
		return a.first == b.first && a.second == b.second;
	}
};

// returns the same ordering for (A,B) and (B,A) so we don't store duplicates
std::pair<Entity, Entity> MakeOrderedPair(const Entity& a, const Entity& b) {
	if (a.id < b.id) {
		return { a, b };
	}
	if (b.id < a.id) {
		return { b, a };
	}

	// IDs are twins, so compare generations to pick who goes first
	if (a.generation <= b.generation) {
		return { a, b };
	}
	return { b, a };
}

void CollisionSystem::Init() {
	ComponentMask mask;
	mask.set(Engine::GetTypeID<ColliderComponent>());
	mask.set(Engine::GetTypeID<TransformComponent>());
    mask.set(Engine::GetTypeID<BoundsComponent>());
	addEntityView(mask);
}

void CollisionSystem::Update(float deltaTime) {
	BroadPhase(deltaTime);
	NarrowPhase(deltaTime);
}

void CollisionSystem::Shutdown() {
}

void CollisionSystem::BroadPhase(float deltaTime) {
	//clear the previous call
	m_candidatePairs.clear();

	if (m_entityViews.empty()) {
		return;
	}


	std::unordered_set<std::pair<Entity, Entity>, EntityPairHash, EntityPairEqual> uniquePairs;
	auto& entities = m_entityViews[0].dense();

	for (const Entity& entity : entities) {
		// iterate through all entities with a collider and query any entities that collide with its collider
		const BoundsComponent& boundsComp = Engine::GetComponent<BoundsComponent>(entity);
		const TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
		const AABB query =
			boundsComp.bounds.computeWorldBounds(transform.position, transform.rotation, transform.scale);
		const std::vector<Entity> nearbyEntities = Engine::QueryAABB(query);

		for (const Entity& other : nearbyEntities) {
			if (entity == other) {
				continue;
			}

			//only create a collision pair if the other also has a collider!
			if (!Engine::HasComponent<ColliderComponent>(other)) {
				continue;
			}

			const auto orderedPair = MakeOrderedPair(entity, other);
			if (uniquePairs.insert(orderedPair).second) {
				m_candidatePairs.push_back(orderedPair);
			}
		}
	}
}

void CollisionSystem::NarrowPhase(float deltaTime) {
	for (const auto& pair: m_candidatePairs) {
		// for all collisions, compute the world collider of each collider
		// since they are stored in local space by default
		const ColliderComponent c_first = ComputeWorldCollider(pair.first);
		const ColliderComponent c_second = ComputeWorldCollider(pair.second);

		ContactInfo contactInfo;
		if (BuildContactInfo(c_first, c_second, contactInfo)) {
			CollisionEvent event;
			event.entities = pair;
			event.normal = contactInfo.normal;
			event.penetrationDepth = contactInfo.penetration;

			// notify observsers!
			NotifyObservers(EventType::COLLISION, event);
		}
	}
}

ColliderComponent CollisionSystem::ComputeWorldCollider(Entity entity) const {
	const ColliderComponent& localCollider = Engine::GetComponent<ColliderComponent>(entity);
	const TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
	ColliderComponent worldCollider = localCollider;

	if (localCollider.isSphere()) {
		Sphere worldSphere =
			localCollider.sphere.computeWorldSphere(transform.position, transform.rotation, transform.scale);
		worldCollider.sphere = worldSphere;
	}
	else {
		AABB worldAABB =
			localCollider.computeWorldBounds(transform.position, transform.rotation, transform.scale);
		worldCollider.box = worldAABB;
	}
	return worldCollider;
}
