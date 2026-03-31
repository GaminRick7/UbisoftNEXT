#pragma once
#include "AABB.h"
#include "math/vec3.h"

struct Sphere {
    Vec3 center;
    float radius;

    Sphere() : center(0.0f, 0.0f, 0.0f), radius(0.0f) {}
    Sphere(const Vec3& c, float r) : center(c), radius(r) {}

    bool contains(const Vec3& point) const;
    bool intersects(const Sphere& other) const;

    AABB computeWorldBounds(const Vec3& pos, const Vec3& rot, const Vec3& scale) const;
    Sphere computeWorldSphere(const Vec3& pos, const Vec3& rot, const Vec3& scale) const;
};
