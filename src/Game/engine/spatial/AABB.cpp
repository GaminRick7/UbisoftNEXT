#include "AABB.h"
#include "Sphere.h"
#include "Frustum.h"
#include <algorithm>
#include <cmath>

Vec3 AABB::getCenter() const {
    return Vec3(
        (minCorner.x + maxCorner.x) * 0.5f,
        (minCorner.y + maxCorner.y) * 0.5f,
        (minCorner.z + maxCorner.z) * 0.5f
    );
}

Vec3 AABB::getSize() const {
    return Vec3(
        maxCorner.x - minCorner.x,
        maxCorner.y - minCorner.y,
        maxCorner.z - minCorner.z
    );
}

Vec3 AABB::getExtents() const {
    return Vec3(
        (maxCorner.x - minCorner.x) * 0.5f,
        (maxCorner.y - minCorner.y) * 0.5f,
        (maxCorner.z - minCorner.z) * 0.5f
    );
}

bool AABB::contains(const Vec3& point) const {
    return point.x >= minCorner.x && point.x <= maxCorner.x &&
           point.y >= minCorner.y && point.y <= maxCorner.y &&
           point.z >= minCorner.z && point.z <= maxCorner.z;
}

bool AABB::contains(const AABB& other) const {
    return other.minCorner.x >= minCorner.x && other.maxCorner.x <= maxCorner.x &&
           other.minCorner.y >= minCorner.y && other.maxCorner.y <= maxCorner.y &&
           other.minCorner.z >= minCorner.z && other.maxCorner.z <= maxCorner.z;
}

bool AABB::intersects(const AABB& other) const {
    // two AABBs intersect if they overlap on ALL axes
    return (minCorner.x <= other.maxCorner.x && maxCorner.x >= other.minCorner.x) &&
           (minCorner.y <= other.maxCorner.y && maxCorner.y >= other.minCorner.y) &&
           (minCorner.z <= other.maxCorner.z && maxCorner.z >= other.minCorner.z);
}

bool AABB::adjacent(const AABB& other) const {
    //TODO
	const float epsilon = 1e-4f;

	const bool overlapX = (minCorner.x < other.maxCorner.x - epsilon) && (maxCorner.x > other.minCorner.x + epsilon);
	const bool overlapY = (minCorner.y < other.maxCorner.y - epsilon) && (maxCorner.y > other.minCorner.y + epsilon);
	const bool overlapZ = (minCorner.z < other.maxCorner.z - epsilon) && (maxCorner.z > other.minCorner.z + epsilon);

	const bool adjacentX = (std::fabs(maxCorner.x - other.minCorner.x) <= epsilon) ||
		(std::fabs(other.maxCorner.x - minCorner.x) <= epsilon);
	const bool adjacentY = (std::fabs(maxCorner.y - other.minCorner.y) <= epsilon) ||
		(std::fabs(other.maxCorner.y - minCorner.y) <= epsilon);
	const bool adjacentZ = (std::fabs(maxCorner.z - other.minCorner.z) <= epsilon) ||
		(std::fabs(other.maxCorner.z - minCorner.z) <= epsilon);

	return (adjacentX && overlapY && overlapZ) ||
		(adjacentY && overlapX && overlapZ) ||
		(adjacentZ && overlapX && overlapY);
}

bool AABB::intersects(const Sphere& sphere) const {
    // ffind the closest point on AABB to sphere center
    Vec3 closestPoint;
    closestPoint.x = (std::max)(minCorner.x, (std::min)(sphere.center.x, maxCorner.x));
    closestPoint.y = (std::max)(minCorner.y, (std::min)(sphere.center.y, maxCorner.y));
    closestPoint.z = (std::max)(minCorner.z, (std::min)(sphere.center.z, maxCorner.z));

    // calculate distance from sphere center to closest point
    Vec3 diff = closestPoint - sphere.center;
    float distanceSq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;

    // check if distance is less than sphere radius
    return distanceSq <= (sphere.radius * sphere.radius);
}

bool AABB::intersects(const Frustum& frustum) const {
    // test AABB against all 6 frustum planes
    for (int i = 0; i < 6; i++) {
        const Plane& plane = frustum.planes[i];

        // the positive vertex - the vertex of AABB that is furthest
        // in the direction of the plane normal (on the positive side)
        // we want to maximize the dot product :D
        Vec3 positiveVertex;
        positiveVertex.x = (plane.normal.x >= 0.0f) ? maxCorner.x : minCorner.x;
        positiveVertex.y = (plane.normal.y >= 0.0f) ? maxCorner.y : minCorner.y;
        positiveVertex.z = (plane.normal.z >= 0.0f) ? maxCorner.z : minCorner.z;

        //f the positive vertex is behind the planea so AABB is outside frustum
        float distance = plane.distanceToPoint(positiveVertex);
        if (distance < 0.0f) {
            return false;
        }
    }

    return true;
}

bool AABB::intersects(const Ray& ray, float& outT) const {
    // TODO
	const float epsilon = 1e-6f;
	float tMin = 0.0f;
	float tMax = ray.length;

	auto axisIntersect = [&](float origin, float dir, float minBound, float maxBound) -> bool {
		if (std::fabs(dir) < epsilon) {
			return origin >= minBound && origin <= maxBound;
		}

		const float invD = 1.0f / dir;
		float t1 = (minBound - origin) * invD;
		float t2 = (maxBound - origin) * invD;
		if (t1 > t2) {
			std::swap(t1, t2);
		}

		tMin = (std::max)(tMin, t1);
		tMax = (std::min)(tMax, t2);
		return tMin <= tMax;
	};

	if (!axisIntersect(ray.origin.x, ray.direction.x, minCorner.x, maxCorner.x)) {
		return false;
	}
	if (!axisIntersect(ray.origin.y, ray.direction.y, minCorner.y, maxCorner.y)) {
		return false;
	}
	if (!axisIntersect(ray.origin.z, ray.direction.z, minCorner.z, maxCorner.z)) {
		return false;
	}

	if (tMax < 0.0f) {
		return false;
	}

	const float hitT = tMin >= 0.0f ? tMin : tMax;
	if (hitT > ray.length) {
		return false;
	}

	outT = hitT;
	return true;
}

AABB AABB::computeWorldBounds(const Vec3& pos, const Vec3& rot, const Vec3& scale) const {
	AABB res;
	// build transformation matrix
	const Mat4 transformMatrix =
		Mat4::translation(pos) *
		Mat4::rotation(rot) *
		Mat4::scaling(scale);

	const Vec3 center = getCenter();
	const Vec3 extents = getExtents();

	const Vec3 corners[8] = {
		center + Vec3(-extents.x, -extents.y, -extents.z),
		center + Vec3( extents.x, -extents.y, -extents.z),
		center + Vec3(-extents.x,  extents.y, -extents.z),
		center + Vec3( extents.x,  extents.y, -extents.z),
		center + Vec3(-extents.x, -extents.y,  extents.z),
		center + Vec3( extents.x, -extents.y,  extents.z),
		center + Vec3(-extents.x,  extents.y,  extents.z),
		center + Vec3( extents.x,  extents.y,  extents.z),
	};

	Vec3 worldMin(
		(std::numeric_limits<float>::max)(),
		(std::numeric_limits<float>::max)(),
		(std::numeric_limits<float>::max)()
	);

	Vec3 worldMax(
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest()
	);

	for (const Vec3& corner : corners) {
		const Vec4 transformed = transformMatrix * Vec4(corner.x, corner.y, corner.z, 1.0f);
		worldMin.x = (std::min)(worldMin.x, transformed.x);
		worldMin.y = (std::min)(worldMin.y, transformed.y);
		worldMin.z = (std::min)(worldMin.z, transformed.z);
		worldMax.x = (std::max)(worldMax.x, transformed.x);
		worldMax.y = (std::max)(worldMax.y, transformed.y);
		worldMax.z = (std::max)(worldMax.z, transformed.z);
	}

	res = AABB(worldMin, worldMax);

	return res;
}

AABB AABB::calculateChildBounds(int octantIndex) const {
	Vec3 center = getCenter();

	Vec3 childMin, childMax;
	// octant layout (0-7):
	// 0: left, bottom, back (-X, -Y, -Z)
	// 1: right, bottom, back (+X, -Y, -Z)
	// 2: left, bottom, front (-X, -Y, +Z)
	// 3: right, bottom, front (+X, -Y, +Z)
	// 4: left, top, back (-X, +Y, -Z)
	// 5: right, top, back (+X, +Y, -Z)
	// 6: left, top, front (-X, +Y, +Z)
	// 7: right, top, front (+X, +Y, +Z)

	// determine X bounds (left or right)
	if (octantIndex == 0 || octantIndex == 2 || octantIndex == 4 || octantIndex == 6) {
		// left side
		childMin.x = minCorner.x;
		childMax.x = center.x;
	} else {
		// right side
		childMin.x = center.x;
		childMax.x = maxCorner.x;
	}

	// determine Y bounds (bottom or top)
	if (octantIndex == 0 || octantIndex == 1 || octantIndex == 2 || octantIndex == 3) {
		// bottom
		childMin.y = minCorner.y;
		childMax.y = center.y;
	} else {
		// top
		childMin.y = center.y;
		childMax.y = maxCorner.y;
	}

	// determine Z bounds (back or front)
	if (octantIndex == 0 || octantIndex == 1 || octantIndex == 4 || octantIndex == 5) {
		// back
		childMin.z = minCorner.z;
		childMax.z = center.z;
	} else {
		// front
		childMin.z = center.z;
		childMax.z = maxCorner.z;
	}

	return AABB(childMin, childMax);
}
