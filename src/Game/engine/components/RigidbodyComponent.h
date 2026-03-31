#pragma once

#include "math/vec3.h"

struct RigidbodyComponent
{
	Vec3 velocity = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 acceleration = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 accumulatedForces = Vec3(0.0f, 0.0f, 0.0f);
	float mass = 1.0f;
	float drag = 0.0f;
	bool useGravity = true;
	bool isKinematic = false;
};


