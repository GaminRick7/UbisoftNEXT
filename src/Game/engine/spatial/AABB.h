#pragma once
#include "math/vec3.h"
#include "Ray.h"
#include "engine/components/TransformComponent.h"

struct Sphere;
struct Frustum;

struct AABB {
	Vec3 minCorner;
	Vec3 maxCorner;

	AABB() : minCorner(0.0f, 0.0f, 0.0f), maxCorner(0.0f, 0.0f, 0.0f) {}
	AABB(const Vec3& min, const Vec3& max) : minCorner(min), maxCorner(max) {}
	
	Vec3 getCenter() const;
	Vec3 getSize() const;
	Vec3 getExtents() const;

	bool contains(const Vec3& point) const;
	bool contains(const AABB& other) const;

	bool intersects(const AABB& other) const;
	bool adjacent(const AABB& other) const;

	bool intersects(const Sphere& sphere) const;
	bool intersects(const Frustum& frustum) const;
	bool intersects(const Ray& ray, float& outT) const;

	// AABB util functions
	AABB computeWorldBounds(const Vec3& pos, const Vec3& rot, const Vec3& scale) const;

	// return the child bounds of an octree node given the index
	AABB calculateChildBounds(int octantIndex) const;
};
