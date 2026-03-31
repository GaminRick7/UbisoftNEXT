#pragma once
#include "../spatial/AABB.h"
#include "../spatial/Sphere.h"

struct ColliderComponent {
	// REMINDER (to myself mostly): these bounds are in local space, not world space !!!

	// Types can be expanded to support other colliders such as capsules, etc.
	enum class Type {
		Sphere,
		AABB
	};

	Type type = Type::AABB;
	Sphere sphere{};
	AABB box{};

	bool isSphere() const { return type == Type::Sphere; }
	bool isAABB() const { return type == Type::AABB; }

	// let's put computeWorldBounds in here to avoid too many isSphere/isAABB checks in the game code
	AABB computeWorldBounds(const Vec3& pos, const Vec3& rot, const Vec3& scale) const {
		AABB res;
		if (isSphere()) res = sphere.computeWorldBounds(pos, rot, scale);
		if (isAABB()) res = box.computeWorldBounds(pos, rot, scale);
		return res;
	}
};


