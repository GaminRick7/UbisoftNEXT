#include "Frustum.h"
#include "AABB.h"
#include "engine/components/CameraComponent.h"
#include "math/mat4.h"
#include <cmath>

Frustum Frustum::fromCamera(const CameraComponent& camera, const Mat4& viewMatrix) {
    Frustum frustum;
    
    Mat4 view = viewMatrix;
    Mat4 projection = Mat4::perspective(camera.fov, camera.aspectRatio, camera.nearPlane, camera.farPlane);
    Mat4 viewProj = projection * view;
    
    // extract frustum planes from view-projection matrix using Gribb/Hartmann method
    // reference: https://stackoverflow.com/questions/12836967/extracting-view-frustum-planes-gribb-hartmann-method
    
    // Left plane
    float leftA = viewProj[3] + viewProj[0];
    float leftB = viewProj[7] + viewProj[4];
    float leftC = viewProj[11] + viewProj[8];
    float leftD = viewProj[15] + viewProj[12];
    float leftLength = std::sqrt(leftA * leftA + leftB * leftB + leftC * leftC);
    if (leftLength > 0.0f) {
        leftA /= leftLength; leftB /= leftLength; leftC /= leftLength; leftD /= leftLength;
    }
    frustum.planes[LEFT] = Plane(Vec3(leftA, leftB, leftC), leftD);
    
    // Right plane
    float rightA = viewProj[3] - viewProj[0];
    float rightB = viewProj[7] - viewProj[4];
    float rightC = viewProj[11] - viewProj[8];
    float rightD = viewProj[15] - viewProj[12];
    float rightLength = std::sqrt(rightA * rightA + rightB * rightB + rightC * rightC);
    if (rightLength > 0.0f) {
        rightA /= rightLength; rightB /= rightLength; rightC /= rightLength; rightD /= rightLength;
    }
    frustum.planes[RIGHT] = Plane(Vec3(rightA, rightB, rightC), rightD);
    
    // Bottom plane
    float bottomA = viewProj[3] + viewProj[1];
    float bottomB = viewProj[7] + viewProj[5];
    float bottomC = viewProj[11] + viewProj[9];
    float bottomD = viewProj[15] + viewProj[13];
    float bottomLength = std::sqrt(bottomA * bottomA + bottomB * bottomB + bottomC * bottomC);
    if (bottomLength > 0.0f) {
        bottomA /= bottomLength; bottomB /= bottomLength; bottomC /= bottomLength; bottomD /= bottomLength;
    }
    frustum.planes[BOTTOM] = Plane(Vec3(bottomA, bottomB, bottomC), bottomD);
    
    // Top plane
    float topA = viewProj[3] - viewProj[1];
    float topB = viewProj[7] - viewProj[5];
    float topC = viewProj[11] - viewProj[9];
    float topD = viewProj[15] - viewProj[13];
    float topLength = std::sqrt(topA * topA + topB * topB + topC * topC);
    if (topLength > 0.0f) {
        topA /= topLength; topB /= topLength; topC /= topLength; topD /= topLength;
    }
    frustum.planes[TOP] = Plane(Vec3(topA, topB, topC), topD);
    
    // Near plane
    float nearA = viewProj[3] + viewProj[2];
    float nearB = viewProj[7] + viewProj[6];
    float nearC = viewProj[11] + viewProj[10];
    float nearD = viewProj[15] + viewProj[14];
    float nearLength = std::sqrt(nearA * nearA + nearB * nearB + nearC * nearC);
    if (nearLength > 0.0f) {
        nearA /= nearLength; nearB /= nearLength; nearC /= nearLength; nearD /= nearLength;
    }
    frustum.planes[NEAR_PLANE] = Plane(Vec3(nearA, nearB, nearC), nearD);
    
    // Far plane
    float farA = viewProj[3] - viewProj[2];
    float farB = viewProj[7] - viewProj[6];
    float farC = viewProj[11] - viewProj[10];
    float farD = viewProj[15] - viewProj[14];
    float farLength = std::sqrt(farA * farA + farB * farB + farC * farC);
    if (farLength > 0.0f) {
        farA /= farLength; farB /= farLength; farC /= farLength; farD /= farLength;
    }
    frustum.planes[FAR_PLANE] = Plane(Vec3(farA, farB, farC), farD);
    
    return frustum;
}

bool Frustum::intersects(const AABB& bounds) const {
    for (int i = 0; i < 6; i++) {
        const Plane& plane = planes[i];
        
        // the positive vertex - the vertex of AABB that is furthest
        // in the direction of the plane normal (on the positive side)
        // we want to maximize the dot product :D
        Vec3 positiveVertex;
        positiveVertex.x = (plane.normal.x >= 0.0f) ? bounds.maxCorner.x : bounds.minCorner.x;
        positiveVertex.y = (plane.normal.y >= 0.0f) ? bounds.maxCorner.y : bounds.minCorner.y;
        positiveVertex.z = (plane.normal.z >= 0.0f) ? bounds.maxCorner.z : bounds.minCorner.z;
        
        //f the positive vertex is behind the planea so AABB is outside frustum
        float distance = plane.distanceToPoint(positiveVertex);
        if (distance < 0.0f) {
            return false;
        }
    }
    
    return true;
}

