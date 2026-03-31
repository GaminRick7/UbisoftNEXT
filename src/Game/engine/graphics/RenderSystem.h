#pragma once
#include <engine/components/CameraComponent.h>
#include <vector>

#include "Model.h"
#include "Particle.h"
#include "engine/components/TransformComponent.h"
#include "engine/ecs/core/System.h"
#include "engine/ecs/core/Entity.h"
#include "math/mat4.h"

struct AABB;
struct LightComponent;

class RenderSystem : public System {
public:
	void Init();
	void Update(float deltaTime);
	void Render();
	void Shutdown();
	
	void SetActiveCamera(Entity cameraEntity);
	bool IsActiveCamera(Entity cameraEntity) const;
	
	// Debug visualization
	static void DrawAABBWireframe(const AABB& bounds, const Mat4& viewProj, float r = 1.0f, float g = 0.0f, float b = 0.0f);
	static void DrawLineWorld(const Vec3& start, const Vec3& end, const Mat4& viewProj, float r = 1.0f, float g = 1.0f, float b = 1.0f);
	
private:
	void renderMesh(const Model& model, const Vec3& color, const Mat4& mvp,
		const Mat4& modelMatrix, const std::vector<Entity>* affectingLights);
	Vec3 calculateLighting(const Vec3& position, const Vec3& normal, const Vec3& baseColor,
		const std::vector<Entity>* affectingLights) const;
	float computeLightRadius(const LightComponent& light) const;
	void renderParticle(Particle& particle);
	CameraComponent* findActiveCamera();

	CameraComponent* m_currentCamera; // Camera used for rendering

	int m_numRendered;
};
