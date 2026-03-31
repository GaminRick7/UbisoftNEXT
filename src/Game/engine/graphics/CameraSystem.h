#pragma once
#include "Model.h"
#include "engine/components/CameraComponent.h"
#include "engine/ecs/core/System.h"
#include "engine/ecs/core/Entity.h"
#include "math/mat4.h"

class CameraSystem : public System {
public:
	void Init();
	void Update(float deltaTime);
	void Shutdown();

	void SetTarget(Entity* target);
	void ClearTarget();
private:
	Entity* m_target;

	void FreeUpdate(float deltaTime, CameraComponent& camera);
	void FollowTargetBoomUpdate(float deltaTime, CameraComponent& camera);
	void OrbitalUpdate(float deltaTime, CameraComponent& camera);
	void TopDownUpdate(float deltaTime, CameraComponent& camera);
};
