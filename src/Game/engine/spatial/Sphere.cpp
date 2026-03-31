#include "Sphere.h"
#include "math/vec3.h"
#include <cmath>

#include "math/mat4.h"

bool Sphere::contains(const Vec3& point) const {
    Vec3 diff = point - center;
    float distanceSq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
    return distanceSq <= (radius * radius);
}

bool Sphere::intersects(const Sphere& other) const {
    Vec3 diff = center - other.center;
    float distanceSq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
    float radiusSum = radius + other.radius;
    return distanceSq <= (radiusSum * radiusSum);
}

AABB Sphere::computeWorldBounds(const Vec3& pos, const Vec3& rot, const Vec3& scale) const {
    // build transformation matrix
    const Mat4 transformMatrix =
        Mat4::translation(pos) *
        Mat4::rotation(rot) *
        Mat4::scaling(scale);
    const Vec4 localCenter(center.x, center.y,
        center.z, 1.0f);
    const Vec4 worldCenter = transformMatrix * localCenter;
    const float maxScale = (std::max)({ std::fabs(scale.x), std::fabs(scale.y), std::fabs(scale.z) });
    const float scaledRadius = radius * maxScale;

    const Vec3 minPoint(
        worldCenter.x - scaledRadius,
        worldCenter.y - scaledRadius,
        worldCenter.z - scaledRadius
    );

    const Vec3 maxPoint(
        worldCenter.x + scaledRadius,
        worldCenter.y + scaledRadius,
        worldCenter.z + scaledRadius
    );

    return AABB(minPoint, maxPoint);
}

Sphere Sphere::computeWorldSphere(const Vec3& pos, const Vec3& rot, const Vec3& scale) const {
    Sphere res;

    // build transformation matric
    const Mat4 transformMatrix =
        Mat4::translation(pos) *
        Mat4::rotation(rot) *
        Mat4::scaling(scale);
    const Vec4 localCenter(center.x, center.y,
            center.z, 1.0f);

    const Vec4 worldCenter = transformMatrix * localCenter;
    res.center = Vec3(worldCenter.x, worldCenter.y, worldCenter.z);

    const float sx = std::abs(scale.x);
    const float sy = std::abs(scale.y);
    const float sz = std::abs(scale.z);
    const float maxScale = (std::max)(sx, (std::max)(sy, sz));

    res.radius = radius * maxScale;

    return res;
}
