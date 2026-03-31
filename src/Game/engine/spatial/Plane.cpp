#include "Plane.h"
#include "math/vec3.h"
#include <cmath>

Plane Plane::fromPointAndNormal(const Vec3& point, const Vec3& normal) {
    Vec3 normalizedNormal = normal.normalized();
    float d = -normalizedNormal.dot(point);
    return Plane(normalizedNormal, d);
}

float Plane::distanceToPoint(const Vec3& point) const {
    return normal.dot(point) + distance;
}

int Plane::classifyPoint(const Vec3& point) const {
    float dist = distanceToPoint(point);
    const float epsilon = 0.0001f;
    if (dist > epsilon) return 1;
    if (dist < -epsilon) return -1;
    return 0;
}

