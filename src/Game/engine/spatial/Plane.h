#pragma once
#include "math/vec3.h"
#include "math/vec4.h"

struct Plane {
    // storing  a plane as a normal (a, b, c) and distance offset from the origin along the normal (d)
    Vec3 normal;
    float distance; 

    Plane() : normal(0.0f, 1.0f, 0.0f), distance(0.0f) {}
    Plane(const Vec3& n, float d) : normal(n.normalized()), distance(d) {}
    
    // construct plane from normal and point on plane
    static Plane fromPointAndNormal(const Vec3& point, const Vec3& normal);
    
    // Distance from point to plane (positive = in front, negative = behind) -- soooo it's really just displacement ;)
    float distanceToPoint(const Vec3& point) const;
    
    // classify point: 1 = front, -1 = back, 0 = on plane
    int classifyPoint(const Vec3& point) const;
};

