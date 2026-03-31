#pragma once
#include "Plane.h"
#include "math/mat4.h"

struct AABB;
struct CameraComponent;

struct Frustum {
    // frustum is just 6 planes (left, right, top, bottom, near, and far)
    Plane planes[6];
    
    enum PlaneIndex {
        LEFT = 0,
        RIGHT = 1,
        TOP = 2,
        BOTTOM = 3,
        NEAR_PLANE = 4,
        FAR_PLANE = 5
    };
    
    // construct frustum from camera and view matrix
    static Frustum fromCamera(const CameraComponent& camera, const Mat4& viewMatrix);
    
    bool intersects(const AABB& bounds) const;
};

