#pragma once
#include "AppSettings.h"
#include "math/vec3.h"

struct CameraComponent
{
	enum class Mode
	{
		Free,
		FollowTargetBoom,
		Orbital,
		TopDown
	};

	// view parameters
	Vec3 position;
	Vec3 target;
	Vec3 up;

	// Projection parameters
	float fov;
	float aspectRatio;
	float nearPlane;
	float farPlane;

	Mode mode;
	bool isActive; // if true, this camera is being used for rendering

	// FollowTargetBoom parameters
	float pivotHeight;
	float lookAtHeight;
	float defaultDistance;
	float minDistance;
	float maxDistance;
	float yawSpeed;
	float pitchSpeed;
	float minPitch;
	float maxPitch;
	float inputDeadZone;

	// Orbital parameters
	float orbitRadius;
	float orbitHeight;
	float orbitSpeed;
	float orbitSmooth;
	float orbitLookAtHeight;
	float orbitYaw;
	Vec3 orbitCenter;
	bool orbitCenterInitialized;

	// Top-down parameters
	float topDownHeight;
	float topDownSmooth;
	float topDownYaw;
	float topDownYawSpeed;
	
	CameraComponent()
		: position(0.0f, 0.0f, 0.0f)
		, target(0.0f, 0.0f, -1.0f)
		, up(0.0f, 1.0f, 0.0f)
		, fov(90.0f)
		, aspectRatio((float) APP_VIRTUAL_WIDTH / (float)APP_VIRTUAL_HEIGHT)
		, nearPlane(0.1f)
		, farPlane(1000.0f)
		, mode(Mode::Free)
		, isActive(true)
		, pivotHeight(1.5f)
		, lookAtHeight(1.0f)
		, defaultDistance(20.0f)
		, minDistance(3.5f)
		, maxDistance(35.0f)
		, yawSpeed(2.0f)
		, pitchSpeed(2.0f)
		, minPitch(-0.35f)
		, maxPitch(1.25f)
		, inputDeadZone(0.15f)
		, orbitRadius(40.0f)
		, orbitHeight(20.0f)
		, orbitSpeed(0.35f)
		, orbitSmooth(6.0f)
		, orbitLookAtHeight(0.0f)
		, orbitYaw(0.0f)
		, orbitCenter(0.0f, 0.0f, 0.0f)
		, orbitCenterInitialized(false)
		, topDownHeight(60.0f)
		, topDownSmooth(6.0f)
		, topDownYaw(0.0f)
		, topDownYawSpeed(0.3f)
	{}
};
