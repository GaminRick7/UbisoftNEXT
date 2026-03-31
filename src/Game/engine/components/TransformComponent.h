#pragma once
#include "math/vec3.h"

struct TransformComponent
{
	Vec3 position = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 rotation = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 scale = Vec3(1.0f, 1.0f, 1.0f);
};