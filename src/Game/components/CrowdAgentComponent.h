#pragma once
#include "math/vec3.h"

struct CrowdAgentComponent {
	Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 velocity = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 flow = Vec3(0,0,0);
	float panic = 0.0f;
	float obedience = 1.0f;
	int groupId = -1;
	float hoverDistanceRemaining = 0.0f;
	float hoverBaseY = 0.0f;
	bool reachedGoal = false;
};
