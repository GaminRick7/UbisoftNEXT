#pragma once
#include "math/vec3.h"

struct Particle {
	Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 velocity = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 color = Vec3(1.0f, 1.0f, 1.0f);
	float alpha = 1.0f;
	float size = 1.0f;
	float age = 0.0f;
	float lifetime = 1.0f;
	bool alive = false;
};
