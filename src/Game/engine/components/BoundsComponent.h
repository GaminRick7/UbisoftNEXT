#pragma once
#include "../spatial/AABB.h"

// bounds component stores AABB for an entity
// used for spatial partitioning
struct BoundsComponent {
    // REMINDER (to myself mostly): these bounds are in local space, not world space !!!
    AABB bounds;
    
    BoundsComponent() : bounds() {}
    BoundsComponent(const AABB& b) : bounds(b) {}
};

