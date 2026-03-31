#pragma once
#include "math/vec3.h"

struct LightComponent
{
	//
	enum class Type {
		Point,

		//TODO
		Global
	};

	// point light valuse
	Vec3 color; // RGB colour
	float intensity; // brightness
	float constant; // attenuation constant term
	float linear; // attenuation linear term
	float quadratic; // attenuation quadratic term
	
	LightComponent()
		: color(1.0f, 1.0f, 1.0f) // wite by default
		, intensity(1.0f)
		, constant(1.0f)
		, linear(0.09f)
		, quadratic(0.032f)
	{}
};

