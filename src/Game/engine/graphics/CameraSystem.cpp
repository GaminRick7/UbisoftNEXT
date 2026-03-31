#include "CameraSystem.h"
#include "engine/Engine.h"
#include "engine/components/TransformComponent.h"
#include "../../../ContestAPI/app.h"
#include "math/vec3.h"
#include "math/MathUtils.h"
#include "engine/spatial/Ray.h"
#include <algorithm>
#include <cmath>

void CameraSystem::Init() {
	m_target = nullptr;
	// entity view for cameras
	ComponentMask cameraMask;
	cameraMask.set(Engine::GetTypeID<CameraComponent>());
	addEntityView(cameraMask);
}

void CameraSystem::Update(float deltaTime) {
	// get all camera entities
	auto& cameraEntities = m_entityViews[0].dense();
	
	// update all active cameras
	for (const Entity& entity : cameraEntities) {
		CameraComponent& camera = Engine::GetComponent<CameraComponent>(entity);
		if (!camera.isActive) {
			continue;
		}
		if (camera.mode == CameraComponent::Mode::Free) {
			FreeUpdate(deltaTime, camera);
		}
		if (camera.mode == CameraComponent::Mode::FollowTargetBoom) {
			FollowTargetBoomUpdate(deltaTime, camera);
		}
		if (camera.mode == CameraComponent::Mode::Orbital) {
			OrbitalUpdate(deltaTime, camera);
		}
		if (camera.mode == CameraComponent::Mode::TopDown) {
			TopDownUpdate(deltaTime, camera);
		}
	}
}

void CameraSystem::Shutdown() {
	// Cleanup if needed
}

void CameraSystem::SetTarget(Entity* target) {
	m_target = target;
}

void CameraSystem::ClearTarget() {
	m_target = nullptr;
}

void CameraSystem::FreeUpdate(float deltaTime, CameraComponent& camera) {
	Vec3 forward = (camera.target - camera.position).normalized();
	if (forward.length() == 0.0f) {
		forward = Vec3(0.0f, 0.0f, -1.0f);
	}

	const Vec3 worldUp(0.0f, 1.0f, 0.0f);
	Vec3 up = camera.up.length() == 0.0f ? worldUp : camera.up.normalized();
	Vec3 right = forward.cross(up).normalized();
	if (right.length() == 0.0f) {
		right = forward.cross(worldUp).normalized();
	}

	const float moveSpeed = 7.0f;
	const float turnSpeed = 3.0f;

	// convert deltaTime from milliseconds to seconds
	const float deltaTimeSeconds = deltaTime / 1000.0f;
	const CController& controller = App::GetController(0);

	// check for key presses and move camera
	Vec3 movement(0.0f, 0.0f, 0.0f);
	if (App::IsKeyPressed(App::KEY_W)) {
		movement = movement + forward * moveSpeed * deltaTimeSeconds;
	}
	if (App::IsKeyPressed(App::KEY_S)) {
		movement = movement - forward * moveSpeed * deltaTimeSeconds;
	}
	if (App::IsKeyPressed(App::KEY_A)) {
		movement = movement - right * moveSpeed * deltaTimeSeconds;
	}
	if (App::IsKeyPressed(App::KEY_D)) {
		movement = movement + right * moveSpeed * deltaTimeSeconds;
	}

	if (controller.CheckButton(App::BTN_Y)) {
		camera.fov = 90.0f;
	}

	const float leftStickDeadZone = 0.2f;
	const float leftStickX = controller.GetLeftThumbStickX();
	const float leftStickY = controller.GetLeftThumbStickY();
	if (std::fabs(leftStickY) > leftStickDeadZone) {
		movement = movement - forward * (leftStickY * moveSpeed * deltaTimeSeconds);
	}
	if (std::fabs(leftStickX) > leftStickDeadZone) {
		movement = movement + right * (leftStickX * moveSpeed * deltaTimeSeconds);
	}

	camera.position = camera.position + movement;

	const float rightStickDeadZone = 0.15f;
	const float rightStickX = controller.GetRightThumbStickX();
	const float rightStickY = controller.GetRightThumbStickY();
	bool orientationChanged = false;
	Vec3 currentForward = forward;
	Vec3 currentUp = up;
	if (std::fabs(rightStickX) > rightStickDeadZone) {
		const float yawAngle = -rightStickX * turnSpeed * deltaTimeSeconds;
		currentForward = rotateAroundAxis(currentForward, worldUp, yawAngle);
		currentUp = rotateAroundAxis(currentUp, worldUp, yawAngle);
		orientationChanged = true;
	}

	Vec3 currentRight = currentForward.cross(currentUp).normalized();
	if (currentRight.length() == 0.0f) {
		currentRight = right;
	}

	if (std::fabs(rightStickY) > rightStickDeadZone) {
		const float pitchAngle = -rightStickY * turnSpeed * deltaTimeSeconds;
		const float maxPitch = 1.25f;
		const float proposedPitch = std::asin(clampFloat(currentForward.y, -1.0f, 1.0f)) + pitchAngle;
		if (proposedPitch >= -maxPitch && proposedPitch <= maxPitch) {
			currentForward = rotateAroundAxis(currentForward, currentRight, pitchAngle);
			currentUp = rotateAroundAxis(currentUp, currentRight, pitchAngle);
			orientationChanged = true;
		}
	}

	if (orientationChanged) {
		forward = currentForward.normalized();
		up = currentUp.normalized();
		right = forward.cross(up).normalized();
		if (right.length() == 0.0f) {
			right = forward.cross(worldUp).normalized();
		}
		camera.up = up;
	}

	// update target to maintain the same relative direction
	camera.target = camera.position + forward;
}

void CameraSystem::FollowTargetBoomUpdate(float deltaTime, CameraComponent& camera) {
	if (!m_target || !Engine::HasComponent<TransformComponent>(*m_target)) {
		// no valid target so fall back to free mode
		FreeUpdate(deltaTime, camera);
		return;
	}

	const TransformComponent& targetTransform = Engine::GetComponent<TransformComponent>(*m_target);

	const float deltaSeconds = deltaTime / 1000.0f;
	const Vec3 pivotPoint = targetTransform.position + Vec3(0.0f, camera.pivotHeight, 0.0f);
	
	// work in pivot space so boom rotation is around character shoulders
	Vec3 offset = camera.position - pivotPoint;
	float currentDistance = offset.length();
	if (currentDistance == 0.0f) {
		offset = Vec3(0.0f, camera.pivotHeight * 0.5f, camera.defaultDistance);
		currentDistance = offset.length();
	}

	// normalize offset so we only care about direction
	Vec3 offsetDir = offset * (1.0f / currentDistance);
	// convert to spherical coords so we can tweak yaw/pitch directly
	offsetDir.y = clampFloat(offsetDir.y, -1.0f, 1.0f);
	float pitch = std::asin(offsetDir.y);
	float yaw = std::atan2(offsetDir.x, offsetDir.z);

	const CController& controller = App::GetController(0);
	const float rightStickX = controller.GetRightThumbStickX();
	const float rightStickY = controller.GetRightThumbStickY();

	if (std::fabs(rightStickX) > camera.inputDeadZone) {
		// spin around target on yaw
		yaw += -rightStickX * camera.yawSpeed * deltaSeconds;
	}
	if (std::fabs(rightStickY) > camera.inputDeadZone) {
		// orbit up/down on pitch
		pitch += -rightStickY * camera.pitchSpeed * deltaSeconds;
	}
	pitch = clampFloat(pitch, camera.minPitch, camera.maxPitch);

	const float desiredDistance = clampFloat(camera.defaultDistance, camera.minDistance, camera.maxDistance);

	// rebuild a direction vector from yaw/pitch so we can place the camera
	const float cosPitch = std::cos(pitch);
	const Vec3 desiredDir(
		std::sin(yaw) * cosPitch,
		std::sin(pitch),
		std::cos(yaw) * cosPitch
	);
	const Vec3 desiredOffset = desiredDir * desiredDistance;
	const Vec3 desiredPosition = pivotPoint + desiredOffset;

	Vec3 finalPosition = desiredPosition;
	if (desiredDistance > 0.0f) {
		// raycast from pivot to camera to keep boom from clipping walls
		const Vec3 rayDir = desiredOffset * (1.0f / desiredDistance);
		const float collisionBuffer = 0.3f;
		Ray boomRay(pivotPoint, rayDir, desiredDistance);
		const auto hits = Engine::QueryRay(boomRay);

		for (const auto& [hitEntity, hitDistance] : hits) {
			if (m_target && hitEntity == *m_target) {
				continue;
			}
			// stop camera just before first collision
			finalPosition = pivotPoint + rayDir * (std::max)(0.0f, hitDistance - collisionBuffer);
			break;
		}
	}

	camera.position = finalPosition;
	
	// always look slightly above the target origin
	camera.target = targetTransform.position + Vec3(0.0f, camera.lookAtHeight, 0.0f);

	// rebuild camera basis
	const Vec3 worldUp(0.0f, 1.0f, 0.0f);
	Vec3 forward = (camera.target - camera.position).normalized();
	if (forward.length() == 0.0f) {
		forward = Vec3(0.0f, 0.0f, -1.0f);
	}
	Vec3 right = forward.cross(worldUp).normalized();
	if (right.length() == 0.0f) {
		right = Vec3(1.0f, 0.0f, 0.0f);
	}
	Vec3 up = right.cross(forward).normalized();
	if (up.length() == 0.0f) {
		up = worldUp;
	}
	camera.up = up;
}

// TODO Add explantion
void CameraSystem::OrbitalUpdate(float deltaTime, CameraComponent& camera) {
	const float deltaSeconds = deltaTime / 1000.0f;
	if (!camera.orbitCenterInitialized) {
		camera.orbitCenter = camera.target;
		camera.orbitCenterInitialized = true;
	}
	const Vec3 orbitCenter = camera.orbitCenter;

	camera.orbitYaw += camera.orbitSpeed * deltaSeconds;
	const float desiredX = orbitCenter.x + std::cos(camera.orbitYaw) * camera.orbitRadius;
	const float desiredZ = orbitCenter.z + std::sin(camera.orbitYaw) * camera.orbitRadius;
	const Vec3 desiredPosition(desiredX, orbitCenter.y + camera.orbitHeight, desiredZ);

	const float smoothT = clampFloat(camera.orbitSmooth * deltaSeconds, 0.0f, 1.0f);
	camera.position = lerp(camera.position, desiredPosition, smoothT);

	camera.target = orbitCenter + Vec3(0.0f, camera.orbitLookAtHeight, 0.0f);
	const Vec3 worldUp(0.0f, 1.0f, 0.0f);
	Vec3 forward = (camera.target - camera.position).normalized();
	if (forward.length() == 0.0f) {
		forward = Vec3(0.0f, 0.0f, -1.0f);
	}
	Vec3 right = forward.cross(worldUp).normalized();
	if (right.length() == 0.0f) {
		right = Vec3(1.0f, 0.0f, 0.0f);
	}
	camera.up = right.cross(forward).normalized();
	if (camera.up.length() == 0.0f) {
		camera.up = worldUp;
	}
}

void CameraSystem::TopDownUpdate(float deltaTime, CameraComponent& camera) {
	const float deltaSeconds = deltaTime / 1000.0f;
	const Vec3 target = camera.target;
	camera.topDownYaw += camera.topDownYawSpeed * deltaSeconds;
	const Vec3 desiredPosition(
		target.x,
		target.y + camera.topDownHeight,
		target.z
	);
	const float smoothT = clampFloat(camera.topDownSmooth * deltaSeconds, 0.0f, 1.0f);
	camera.position = lerp(camera.position, desiredPosition, smoothT);

	camera.target = target;
	camera.up = Vec3(std::sin(camera.topDownYaw), 0.0f, std::cos(camera.topDownYaw));
}



