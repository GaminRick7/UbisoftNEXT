#pragma once

#include <cmath>
#include "vec3.h"

// TODO
// Generic linear interpolation helper for types supporting +, -, and scalar *.
template <typename T>
inline T lerp(const T& a, const T& b, float t) {
	return a + (b - a) * t;
}

inline float clampFloat(float value, float minValue, float maxValue) {
	if (value < minValue) {
		return minValue;
	}
	if (value > maxValue) {
		return maxValue;
	}
	return value;
}

inline Vec3 rotateAroundAxis(const Vec3& vec, const Vec3& axis, float angle) {
	if (angle == 0.0f) {
		return vec;
	}

	const float axisLength = axis.length();
	if (axisLength == 0.0f) {
		return vec;
	}

	const Vec3 normalizedAxis = axis * (1.0f / axisLength);
	const float cosAngle = std::cos(angle);
	const float sinAngle = std::sin(angle);
	return vec * cosAngle
		+ normalizedAxis.cross(vec) * sinAngle
		+ normalizedAxis * (normalizedAxis.dot(vec) * (1.0f - cosAngle));
}


