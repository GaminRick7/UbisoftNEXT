#pragma once
#include "math/vec3.h"
#include <vector>


struct AIComponent
{
	std::vector<Vec3> path;
	size_t currentPathIndex = 0;
	float speed = 15.0f;
	float arriveDistance = 0.5f;
	Vec3 velocity = Vec3(0.0f, 0.0f, 0.0f);
	float turnRate = 6.0f;
	bool isMoving = true;
	bool stopOnArrival = false;
	bool isHeldForDetection = false;
	float noDetectionTimer = 0.0f;
	Vec3 heldCellCenter = Vec3(0.0f, 0.0f, 0.0f);
	bool drawPath = true;
};
