#include "PhysicsSystem.h"

#include <algorithm>

#include "engine/Engine.h"
#include "engine/components/RigidbodyComponent.h"
#include "engine/components/TransformComponent.h"
#include "engine/physics/CollisionEvent.h"
#include "engine/physics/CollisionSystem.h"

void PhysicsSystem::Init() {
	ComponentMask mask;
	mask.set(Engine::GetTypeID<TransformComponent>());
	mask.set(Engine::GetTypeID<RigidbodyComponent>());

	addEntityView(mask);

	Engine::GetSystem<CollisionSystem>().AddObserver(this);
}

void PhysicsSystem::Update(float deltaTime) {
	const float deltaSeconds = deltaTime * 0.001f;
	if (m_entityViews.empty()) {
		return;
	}

	static const Vec3 kGravity(0.0f, -9.81f, 0.0f);
	static const float kSleepThresholdSq = 1e-6f;

	auto& entities = m_entityViews[0].dense();
	for (const Entity& entity : entities) {
		auto& transform = Engine::GetComponent<TransformComponent>(entity);
		auto& rigidbody = Engine::GetComponent<RigidbodyComponent>(entity);

		// kinematic so we shouldnt do anything
		if (rigidbody.isKinematic) {
			rigidbody.accumulatedForces = Vec3(0.0f, 0.0f, 0.0f);
			rigidbody.acceleration = Vec3(0.0f, 0.0f, 0.0f);
			continue;
		}

		// get total force from rigidbody component then apply garvity
		Vec3 totalForce = rigidbody.accumulatedForces;
		if (rigidbody.useGravity) {
			totalForce = totalForce + (gravity * rigidbody.mass);
		}

		// calculate acceleration from a = F / m 
		Vec3 acceleration(0.0f, 0.0f, 0.0f);
		if (rigidbody.mass > 0.0f) {
			const float invMass = 1.0f / rigidbody.mass;
			acceleration = totalForce * invMass;
		}

		// factor in drag
		if (rigidbody.drag > 0.0f) {
			acceleration = acceleration - (rigidbody.velocity * rigidbody.drag);
		}

		// get velocity from v = v0 + at
		rigidbody.velocity = rigidbody.velocity + (acceleration * deltaSeconds);

		// if velocity is super low, then we clamp it to 0
		const float speedSq =
			rigidbody.velocity.x * rigidbody.velocity.x +
			rigidbody.velocity.y * rigidbody.velocity.y +
			rigidbody.velocity.z * rigidbody.velocity.z;
		if (speedSq < kSleepThresholdSq) {
			rigidbody.velocity = Vec3(0.0f, 0.0f, 0.0f);
		}

		//update position based on velocity
		transform.position = transform.position + (rigidbody.velocity * deltaSeconds);

		//clear consumed forces so the next frame starts fresh
		rigidbody.acceleration = acceleration;
		rigidbody.accumulatedForces = Vec3(0.0f, 0.0f, 0.0f);
	}
}

void PhysicsSystem::Shutdown() {
	Engine::GetSystem<CollisionSystem>().RemoveObserver(this);
}

void PhysicsSystem::OnNotify(EventType type, const Event& payload) {
	if (type != EventType::COLLISION) {
		return;
	}

	// cast to CollisionEvent* so we can extract the payload
	const CollisionEvent* collisionEvent = dynamic_cast<const CollisionEvent*>(&payload);
	if (!collisionEvent) {
		return;
	}

	// get the entities involved in the collision's components
	const Entity& first = collisionEvent->entities.first;
	const Entity& second = collisionEvent->entities.second;

	auto& transformA = Engine::GetComponent<TransformComponent>(first);
	auto& transformB = Engine::GetComponent<TransformComponent>(second);
	auto& rigidbodyA = Engine::GetComponent<RigidbodyComponent>(first);
	auto& rigidbodyB = Engine::GetComponent<RigidbodyComponent>(second);

	// if both are kinematic there is nothing to resolve
	if (rigidbodyA.isKinematic && rigidbodyB.isKinematic) {
		return;
	}

	Vec3 normal = collisionEvent->normal;

	const Vec3 relativeVelocity = rigidbodyB.velocity - rigidbodyA.velocity;
	const float velocityAlongNormal = relativeVelocity.dot(normal);

	// already separating so wecan skip
	if (velocityAlongNormal > 0.0f) {
		return;
	}

	// restitution (e) controls bounciness (setting it to 0.2 so its mostly inelastic)
	const float restitution = 0.7f;
	const float invMassA = (!rigidbodyA.isKinematic && rigidbodyA.mass > 0.0f) ? (1.0f / rigidbodyA.mass) : 0.0f;
	const float invMassB = (!rigidbodyB.isKinematic && rigidbodyB.mass > 0.0f) ? (1.0f / rigidbodyB.mass) : 0.0f;
	const float invMassSum = invMassA + invMassB;
	if (invMassSum <= 0.0f) {
		return;
	}

	// impulse scalar j = -(1 + e) * vAB / (invMassSum)
	const float impulseScalar = -(1.0f + restitution) * velocityAlongNormal / invMassSum;
	const Vec3 impulse = normal * impulseScalar;

	if (invMassA > 0.0f) {
		rigidbodyA.velocity = rigidbodyA.velocity - (impulse * invMassA);
	}

	if (invMassB > 0.0f) {
		rigidbodyB.velocity = rigidbodyB.velocity + (impulse * invMassB);
	}

	// penetration position correction so that objects don't sink into each other over time
	// small overlaps under 0.005 are ignored to avoid jitter
	const float penetrationSlop = 0.005f;
	// only 80% of the overlap is fixed to avoid overcorrection
	const float penetrationPercent = 0.8f;
	const float penetrationDepth = (std::max)(collisionEvent->penetrationDepth - penetrationSlop, 0.0f);

	if (invMassSum > 0.0f && penetrationDepth > 0.0f) {
		const Vec3 correction = normal * (penetrationDepth * (penetrationPercent / invMassSum));
		if (invMassA > 0.0f) {
			transformA.position = transformA.position - (correction * invMassA);
		}

		if (invMassB > 0.0f) {
			transformB.position = transformB.position + (correction * invMassB);
		}
	}
}

