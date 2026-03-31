#pragma once

#include "math/vec3.h"

struct Ray
{
	Vec3 origin;
	Vec3 direction; // should be normalized where possible
	float length;

	Ray()
		: origin(0.0f, 0.0f, 0.0f)
		, direction(0.0f, 0.0f, -1.0f)
		, length(0.0f)
	{}

	Ray(const Vec3& o, const Vec3& d, float len)
		: origin(o)
		, direction(d)
		, length(len)
	{}
};

