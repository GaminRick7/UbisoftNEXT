//#define _DEBUG
#include "RenderSystem.h"

#include "engine/Engine.h"
#include "engine/components/MeshComponent.h"
#include "engine/components/TransformComponent.h"
#include "engine/components/CameraComponent.h"
#include "engine/components/LightComponent.h"
#include "components/CrowdAgentComponent.h"
#include "components/CrowdGroupComponent.h"
#include "components/AIComponent.h"
#include "engine/spatial/Frustum.h"
#include "engine/spatial/AABB.h"
#include "engine/spatial/Sphere.h"
#include "../ecs/core/constants.h"
#include "../../../ContestAPI/app.h"
#include "../../../ContestAPI/AppSettings.h"
#include "math/vec4.h"
#include "math/vec3.h"
#include <unordered_map>
#include <algorithm>
#include <unordered_set>
#include <cmath>
#include <array>
#include <vector>

#include "AI/NavSystem.h"
#include "systems/CrowdSimulationSystem.h"
#include "systems/FlowFieldSystem.h"

constexpr float kNearPlaneEpsilon = 1e-6f;

//TODO constants for maximum light distance + falloff
constexpr float kMaxLightDistance = 80.0f;
constexpr float kLightFadeStartRatio = 0.8f;

struct VertexData {
	Vec4 clip;
	Vec3 world;

	VertexData()
		: clip(0.0f, 0.0f, 0.0f, 0.0f)
		, world(0.0f, 0.0f, 0.0f) {}

	VertexData(const Vec4& clipIn, const Vec3& worldIn)
		: clip(clipIn)
		, world(worldIn) {}
};

bool isInsideNearPlane(const VertexData& v) {
	return (v.clip.z + v.clip.w) >= 0.0f;
}

VertexData intersectWithNearPlane(const VertexData& a, const VertexData& b) {
	const float distA = a.clip.z + a.clip.w;
	const float distB = b.clip.z + b.clip.w;
	const float denom = distA - distB;
	float t = 0.0f;
	if (std::fabs(denom) > kNearPlaneEpsilon) {
		t = distA / denom;
	}

	return VertexData(
		a.clip + (b.clip - a.clip) * t,
		a.world + (b.world - a.world) * t
	);
}


void RenderSystem::Init() {
	m_currentCamera = nullptr;
	
	// entity view for renderable entities (Transform + Mesh)
	ComponentMask renderableMask;
	renderableMask.set(Engine::GetTypeID<TransformComponent>());
	renderableMask.set(Engine::GetTypeID<MeshComponent>());
	addEntityView(renderableMask);
	
	// entity view for cameras
	ComponentMask cameraMask;
	cameraMask.set(Engine::GetTypeID<CameraComponent>());
	addEntityView(cameraMask);
	
	// entity view for lights (Transform + Light)
	ComponentMask lightMask;
	lightMask.set(Engine::GetTypeID<TransformComponent>());
	lightMask.set(Engine::GetTypeID<LightComponent>());
	addEntityView(lightMask);

	// entity view for crowd agents (Transform + Mesh + CrowdAgent)
	ComponentMask crowdMask;
	crowdMask.set(Engine::GetTypeID<TransformComponent>());
	crowdMask.set(Engine::GetTypeID<MeshComponent>());
	crowdMask.set(Engine::GetTypeID<CrowdAgentComponent>());
	addEntityView(crowdMask);

	// entity view for AI-driven drones (AIComponent)
	ComponentMask aiMask;
	aiMask.set(Engine::GetTypeID<AIComponent>());
	addEntityView(aiMask);
}

void RenderSystem::Update(float deltaTime) {
	// Update the current camera reference from active camera entities
	m_currentCamera = findActiveCamera();
}

void RenderSystem::Render() {
	if (!m_currentCamera) {
		m_currentCamera = findActiveCamera();
	}

	if (!m_currentCamera) {
		return;
	}


	Mat4 viewMatrix = Mat4::lookAt(m_currentCamera->position, m_currentCamera->target, m_currentCamera->up);
	Mat4 projectionMatrix = Mat4::perspective(
		m_currentCamera->fov,
		m_currentCamera->aspectRatio,
		m_currentCamera->nearPlane,
		m_currentCamera->farPlane
	);
	Mat4 viewProj = projectionMatrix * viewMatrix;
	const Vec3 cameraPos = m_currentCamera->position;

	// crowd render LOD parameters that describe how far each LOD  range is
	const float crowdNearDistSq = CrowdGroupComponent::kCrowdNearDist * CrowdGroupComponent::kCrowdNearDist;
	const float crowdMidDistSq = CrowdGroupComponent::kCrowdMidDist * CrowdGroupComponent::kCrowdMidDist;
	const float crowdFarDistSq = CrowdGroupComponent::kCrowdFarDist * CrowdGroupComponent::kCrowdFarDist;

	Frustum frustum = Frustum::fromCamera(*m_currentCamera, viewMatrix);
	auto visibleEntities = Engine::QueryFrustum(frustum);

	// keep track of which lights actually touch each entity
	std::unordered_map<EntityID, std::vector<Entity>> entityLightMap;
	auto& lightEntities = m_entityViews[2].dense();
	entityLightMap.reserve(visibleEntities.size());

	// build the list once per frame so triangles only evaluate nearby lights
	for (const Entity& lightEntity : lightEntities) {
		TransformComponent& lightTransform = Engine::GetComponent<TransformComponent>(lightEntity);
		LightComponent& lightComp = Engine::GetComponent<LightComponent>(lightEntity);

		const Vec3 lightToCamera = lightTransform.position - cameraPos;
		const float lightDistSq = lightToCamera.dot(lightToCamera);
		const float maxLightDistSq = kMaxLightDistance * kMaxLightDistance;
		if (lightDistSq > maxLightDistSq) {
			continue;
		}

		float lightRadius = computeLightRadius(lightComp);
		if (lightRadius <= 0.0f) {
			continue;
		}

		Sphere influence(lightTransform.position, lightRadius);
		auto affectedEntities = Engine::QuerySphere(influence);

		for (const Entity& affected : affectedEntities) {
			entityLightMap[affected.id].push_back(lightEntity);
		}
	}

	m_numRendered  = 0;

	std::unordered_set<int> visibleCrowdGroups;
	visibleCrowdGroups.reserve(visibleEntities.size());

	for (const Entity& entity : visibleEntities) {
		if (Engine::HasComponent<CrowdGroupComponent>(entity)) {
			const CrowdGroupComponent& group = Engine::GetComponent<CrowdGroupComponent>(entity);
			if (group.groupId >= 0) {
				visibleCrowdGroups.insert(group.groupId);
			}
			continue;
		}

		if (!Engine::HasComponent<MeshComponent>(entity)) {
			continue;
		}

		TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
		MeshComponent& meshComp = Engine::GetComponent<MeshComponent>(entity);
		
		if (!meshComp.mesh) {
			continue;
		}

		Mat4 modelMatrix = Mat4().translate(transform.position).rotate(transform.rotation).scale(transform.scale);

		// build MVP matrix
		Mat4 mvp = viewProj * modelMatrix;

		const auto lightIt = entityLightMap.find(entity.id);
		const std::vector<Entity>* affectingLights = nullptr;
		if (lightIt != entityLightMap.end()) {
			affectingLights = &lightIt->second;
		}

		// render the mesh
		renderMesh(*meshComp.mesh, meshComp.colour, mvp, modelMatrix, affectingLights);

		m_numRendered += 1;
	}

	std::unordered_map<int, size_t> groupCounters;
	auto& crowdEntities = m_entityViews[3].dense();
	// iterate through each member of the crowd and calculate LOD
	// rationale behind LOD system: considering the number of entities in these crowdes, 
	// a way to improve performance is to render fewer entites at further distances
	for (const Entity& entity : crowdEntities) {
		const CrowdAgentComponent& agent = Engine::GetComponent<CrowdAgentComponent>(entity);
		if (agent.groupId < 0 || visibleCrowdGroups.find(agent.groupId) == visibleCrowdGroups.end()) {
			continue;
		}

		TransformComponent& transform = Engine::GetComponent<TransformComponent>(entity);
		MeshComponent& meshComp = Engine::GetComponent<MeshComponent>(entity);
		if (!meshComp.mesh) {
			continue;
		}

		const size_t memberIndex = groupCounters[agent.groupId]++;
		const Vec3 offset = transform.position - cameraPos;
		const float distSq = offset.dot(offset);
		// beyond far so we skip entirely
		if (distSq > crowdFarDistSq) {
			continue;
		}
		// mid-far range so we render 1/4 of members
		if (distSq > crowdMidDistSq && (memberIndex % 4 != 0)) {
			continue;
		}
		// near-mid range so we render 1/2 of members
		if (distSq > crowdNearDistSq && (memberIndex % 2 != 0)) {
			continue;
		}

		// compute MVP and render mesh
		Mat4 modelMatrix = Mat4().translate(transform.position).rotate(transform.rotation).scale(transform.scale);
		Mat4 mvp = viewProj * modelMatrix;
		renderMesh(*meshComp.mesh, meshComp.colour, mvp, modelMatrix, nullptr);
		m_numRendered += 1;
	}

	auto& aiEntities = m_entityViews[4].dense();
	size_t aiIndex = 0;
	for (const Entity& entity : aiEntities) {
		const AIComponent& ai = Engine::GetComponent<AIComponent>(entity);
		if (!ai.drawPath || ai.path.size() < 2) {
			aiIndex += 1;
			continue;
		}

		Vec3 pathColor(0.1f, 0.8f, 1.0f);
		if (Engine::HasComponent<LightComponent>(entity)) {
			const LightComponent& light = Engine::GetComponent<LightComponent>(entity);
			pathColor = light.color;
		}

		const float aiOffset = 0.5f * static_cast<float>(aiIndex);
		const Vec3 offset(aiOffset, aiOffset, aiOffset);
		for (size_t i = 1; i < ai.path.size(); ++i) {
			DrawLineWorld(ai.path[i - 1] + offset, ai.path[i] + offset, viewProj, pathColor.x, pathColor.y, pathColor.z);
		}

		aiIndex += 1;
	}
	// const Vec3 pathStart(0.0f, 5.0f, 0.0f);
	// const Vec3 pathEnd(85.0f, 5.0f, 70.0f);
	// auto& navSystem = Engine::GetSystem<NavSystem>();
	// const auto& pathPoints = navSystem.GetPath(pathStart, pathEnd);
	// for (size_t i = 1; i < pathPoints.size(); ++i) {
	// 	DrawLineWorld(pathPoints[i - 1], pathPoints[i], viewProj, 0.0f, 0.5f, 1.0f);
	// }

#ifdef _DEBUG

	const std::vector<AABB> nodeBounds = Engine::GetOctreeNodeBounds();
	for (const AABB& bounds : nodeBounds) {
		DrawAABBWireframe(bounds, viewProj, 0.0f, 1.0f, 0.0f);
	}

	// Nav SVO bounds (debug)
	const auto& navSystem = Engine::GetSystem<NavSystem>();
	const std::vector<AABB> navBounds = navSystem.GetNodeBounds(false);
	for (const AABB& bounds : navBounds) {
		DrawAABBWireframe(bounds, viewProj, 1.0f, 1.0f, 0.0f);
	}
	const std::vector<AABB> navBlockedBounds = navSystem.GetNodeBounds(true);
	for (const AABB& bounds : navBlockedBounds) {
		DrawAABBWireframe(bounds, viewProj, 1.0f, 0.2f, 0.2f);
	}

	// Flow field cell bounds (debug)
	const auto& flowSystem = Engine::GetSystem<FlowFieldSystem>();
	const int flowWidth = flowSystem.GetFlowWidth();
	const int flowHeight = flowSystem.GetFlowHeight();
	const float flowCellSize = flowSystem.GetCellSize();
	const Vec3& flowOrigin = flowSystem.GetFlowOrigin();
	if (flowWidth > 0 && flowHeight > 0 && flowCellSize > 0.0f) {
		const float yMin = -0.1f;
		const float yMax = 0.1f;
		for (int z = 0; z < flowHeight; ++z) {
			for (int x = 0; x < flowWidth; ++x) {
				const float minX = flowOrigin.x + x * flowCellSize;
				const float minZ = flowOrigin.z + z * flowCellSize;
				const float maxX = minX + flowCellSize;
				const float maxZ = minZ + flowCellSize;
				const bool isHighlightCell = (x == 0 && z == 5);
				const float r = isHighlightCell ? 1.0f : 0.2f;
				const float g = isHighlightCell ? 0.2f : 0.4f;
				const float b = isHighlightCell ? 0.2f : 1.0f;
				DrawAABBWireframe(
					AABB(Vec3(minX, yMin, minZ), Vec3(maxX, yMax, maxZ)),
					viewProj,
					r,
					g,
					b
				);
			}
		}
	}

	App::Print(0,0, std::to_string(m_numRendered).c_str());

	const uint32_t totalEntities = Engine::GetEntityCount();
	const std::string entityCountText = "Entities: " + std::to_string(totalEntities);
	App::Print(APP_VIRTUAL_WIDTH - 200.0f, 10.0f, entityCountText.c_str());

#endif
}

void RenderSystem::renderMesh(const Model& model, const Vec3& color, const Mat4& mvp, const Mat4& modelMatrix, const std::vector<Entity>* affectingLights) {
	//TODO
	// Render each triangle in the model
	for (const auto& triangle : model.triangles) {

		// transform vertices to world space
		Vec4 v0_world = modelMatrix * Vec4(triangle.vertices[0].x, triangle.vertices[0].y, triangle.vertices[0].z, 1.0f);
		Vec4 v1_world = modelMatrix * Vec4(triangle.vertices[1].x, triangle.vertices[1].y, triangle.vertices[1].z, 1.0f);
		Vec4 v2_world = modelMatrix * Vec4(triangle.vertices[2].x, triangle.vertices[2].y, triangle.vertices[2].z, 1.0f);

		Vec3 v0_ws = Vec3(v0_world.x, v0_world.y, v0_world.z);
		Vec3 v1_ws = Vec3(v1_world.x, v1_world.y, v1_world.z);
		Vec3 v2_ws = Vec3(v2_world.x, v2_world.y, v2_world.z);

		// calculate triangle normal in world space
		Vec3 edge1 = v1_ws - v0_ws;
		Vec3 edge2 = v2_ws - v0_ws;
		Vec3 normal = edge1.cross(edge2);
		normal.normalize();

		// vector from camera to triangle
		Vec3 camToTri = v0_ws - m_currentCamera->position;

		// cull backfaces by taking dot product of normal and camera-to-triangle vector
		float dotProd = normal.dot(camToTri);
		if (dotProd >= 0.0f) {
			continue; // Skip backface
		}

		// vertices through MVP matrix for rendering
		Vec4 v0(triangle.vertices[0].x, triangle.vertices[0].y, triangle.vertices[0].z, 1.0f);
		Vec4 v1(triangle.vertices[1].x, triangle.vertices[1].y, triangle.vertices[1].z, 1.0f);
		Vec4 v2(triangle.vertices[2].x, triangle.vertices[2].y, triangle.vertices[2].z, 1.0f);

		// Apply MVP transformation
		Vec4 clip0 = mvp * v0;
		Vec4 clip1 = mvp * v1;
		Vec4 clip2 = mvp * v2;

		std::array<VertexData, 3> originalVerts = {{
			{clip0, v0_ws},
			{clip1, v1_ws},
			{clip2, v2_ws}
		}};

		std::vector<VertexData> clippedVerts;
		clippedVerts.reserve(5);

		VertexData prev = originalVerts.back();
		bool prevInside = isInsideNearPlane(prev);
		for (const VertexData& curr : originalVerts) {
			const bool currInside = isInsideNearPlane(curr);

			if (currInside && prevInside) {
				clippedVerts.push_back(curr);
			} else if (prevInside && !currInside) {
				clippedVerts.push_back(intersectWithNearPlane(prev, curr));
			} else if (!prevInside && currInside) {
				clippedVerts.push_back(intersectWithNearPlane(prev, curr));
				clippedVerts.push_back(curr);
			}

			prev = curr;
			prevInside = currInside;
		}

		if (clippedVerts.size() < 3) {
			continue;
		}

		for (size_t i = 1; i + 1 < clippedVerts.size(); ++i) {
			const VertexData& a = clippedVerts[0];
			const VertexData& b = clippedVerts[i];
			const VertexData& c = clippedVerts[i + 1];

			const float minW = kNearPlaneEpsilon;
			if (std::fabs(a.clip.w) < minW || std::fabs(b.clip.w) < minW || std::fabs(c.clip.w) < minW) {
				continue;
			}

			const float invW0 = 1.0f / a.clip.w;
			const float invW1 = 1.0f / b.clip.w;
			const float invW2 = 1.0f / c.clip.w;

			const float ndc0_x = a.clip.x * invW0;
			const float ndc0_y = a.clip.y * invW0;
			const float ndc0_z = a.clip.z * invW0;
			const float screen0_x = (ndc0_x + 1.0f) * (APP_VIRTUAL_WIDTH / 2.0f);
			const float screen0_y = (ndc0_y + 1.0f) * (APP_VIRTUAL_HEIGHT / 2.0f);

			const float ndc1_x = b.clip.x * invW1;
			const float ndc1_y = b.clip.y * invW1;
			const float ndc1_z = b.clip.z * invW1;
			const float screen1_x = (ndc1_x + 1.0f) * (APP_VIRTUAL_WIDTH / 2.0f);
			const float screen1_y = (ndc1_y + 1.0f) * (APP_VIRTUAL_HEIGHT / 2.0f);

			const float ndc2_x = c.clip.x * invW2;
			const float ndc2_y = c.clip.y * invW2;
			const float ndc2_z = c.clip.z * invW2;
			const float screen2_x = (ndc2_x + 1.0f) * (APP_VIRTUAL_WIDTH / 2.0f);
			const float screen2_y = (ndc2_y + 1.0f) * (APP_VIRTUAL_HEIGHT / 2.0f);

			const Vec3 litColor0 = calculateLighting(a.world, normal, color, affectingLights);
			const Vec3 litColor1 = calculateLighting(b.world, normal, color, affectingLights);
			const Vec3 litColor2 = calculateLighting(c.world, normal, color, affectingLights);

			App::DrawTriangle(
				screen0_x, screen0_y, ndc0_z, 1.0f,
				screen1_x, screen1_y, ndc1_z, 1.0f,
				screen2_x, screen2_y, ndc2_z, 1.0f,
				litColor0.x, litColor0.y, litColor0.z,
				litColor1.x, litColor1.y, litColor1.z,
				litColor2.x, litColor2.y, litColor2.z,
				false
			);
		}
	}
}

bool projectCornerToScreen(const Vec3& point, const Mat4& viewProj, float& outX, float& outY) {
	Vec4 clip = viewProj * Vec4(point.x, point.y, point.z, 1.0f);
	if (clip.w <= 0.0f) {
		return false;
	}

	const float invW = 1.0f / clip.w;
	const float ndcX = clip.x * invW;
	const float ndcY = clip.y * invW;
	const float ndcZ = clip.z * invW;

	if (ndcZ < -1.0f || ndcZ > 1.0f) {
		return false;
	}

	outX = (ndcX + 1.0f) * (APP_VIRTUAL_WIDTH * 0.5f);
	outY = (ndcY + 1.0f) * (APP_VIRTUAL_HEIGHT * 0.5f);
	return true;
}

void RenderSystem::DrawAABBWireframe(const AABB& bounds, const Mat4& viewProj, float r, float g, float b) {
	const Vec3& min = bounds.minCorner;
	const Vec3& max = bounds.maxCorner;

	const Vec3 corners[8] = {
		Vec3(min.x, min.y, min.z),
		Vec3(max.x, min.y, min.z),
		Vec3(min.x, max.y, min.z),
		Vec3(max.x, max.y, min.z),
		Vec3(min.x, min.y, max.z),
		Vec3(max.x, min.y, max.z),
		Vec3(min.x, max.y, max.z),
		Vec3(max.x, max.y, max.z)
	};

	float screenX[8];
	float screenY[8];
	bool projected[8];

	for (int i = 0; i < 8; ++i) {
		projected[i] = projectCornerToScreen(corners[i], viewProj, screenX[i], screenY[i]);
	}

	const int edges[12][2] = {
		{0,1}, {1,3}, {3,2}, {2,0},
		{4,5}, {5,7}, {7,6}, {6,4},
		{0,4}, {1,5}, {2,6}, {3,7}
	};

	for (const auto& edge : edges) {
		const int start = edge[0];
		const int end = edge[1];
		if (projected[start] && projected[end]) {
			App::DrawLine(screenX[start], screenY[start], screenX[end], screenY[end], r, g, b);
		}
	}
}

void RenderSystem::DrawLineWorld(const Vec3& start, const Vec3& end, const Mat4& viewProj, float r, float g, float b) {
	// TODO
	float startX = 0.0f;
	float startY = 0.0f;
	float endX = 0.0f;
	float endY = 0.0f;

	const bool startProjected = projectCornerToScreen(start, viewProj, startX, startY);
	const bool endProjected = projectCornerToScreen(end, viewProj, endX, endY);
	if (startProjected && endProjected) {
		App::DrawLine(startX, startY, endX, endY, r, g, b);
	}
}

void RenderSystem::Shutdown() {
	m_currentCamera = nullptr;
}

void RenderSystem::SetActiveCamera(Entity cameraEntity) {
	// deactivate all cameras
	// m_entityViews[1] is for cameras
	auto& cameraEntities = m_entityViews[1].dense();
	for (const Entity& entity : cameraEntities) {
		CameraComponent& camera = Engine::GetComponent<CameraComponent>(entity);
		camera.isActive = false;
	}
	
	// ativate the specified camera
	CameraComponent& camera = Engine::GetComponent<CameraComponent>(cameraEntity);
	camera.isActive = true;
	m_currentCamera = &camera;
}

bool RenderSystem::IsActiveCamera(Entity cameraEntity) const {
	CameraComponent& camera = Engine::GetComponent<CameraComponent>(cameraEntity);
	return camera.isActive;
}

Vec3 RenderSystem::calculateLighting(const Vec3& position, const Vec3& normal, const Vec3& baseColor, const std::vector<Entity>* affectingLights) const {
	// Start with ambient lighting (minimum visibility)
	const float ambientStrength = 0.08f;
	Vec3 ambient = baseColor * ambientStrength;
	Vec3 finalColor = ambient;

	// TODO: Replace with a proper directional light component.
	// Global directional light (simple, fixed direction)
	const Vec3 sunDir = Vec3(-0.3f, -1.0f, -0.2f).normalized();
	const float sunStrength = 0.35f;
	const Vec3 sunDirNeg = sunDir * -1.0f;
	const float sunDiff = (std::max)(0.0f, normal.dot(sunDirNeg));
	finalColor = finalColor + (baseColor * sunDiff * sunStrength);

	if (!affectingLights) {
		return finalColor;
	}

	// accumulate contribution from nearby point lights only
	for (const Entity& lightEntity : *affectingLights) {
		TransformComponent& lightTransform = Engine::GetComponent<TransformComponent>(lightEntity);
		LightComponent& light = Engine::GetComponent<LightComponent>(lightEntity);
		
		float fade = 1.0f;
		const Vec3 lightToCamera = lightTransform.position - m_currentCamera->position;
		const float lightDistSq = lightToCamera.dot(lightToCamera);
		const float maxLightDistSq = kMaxLightDistance * kMaxLightDistance;
		if (lightDistSq >= maxLightDistSq) {
			continue;
		}
		const float fadeStartDist = kMaxLightDistance * kLightFadeStartRatio;
		const float fadeStartSq = fadeStartDist * fadeStartDist;
		if (lightDistSq > fadeStartSq) {
			const float lightDist = std::sqrt(lightDistSq);
			float t = (lightDist - fadeStartDist) / (kMaxLightDistance - fadeStartDist);
			t = (std::min)((std::max)(t, 0.0f), 1.0f);
			const float smooth = t * t * (3.0f - 2.0f * t);
			fade = 1.0f - smooth;
		}

		// calculate direction from surface to light
		Vec3 lightDir = lightTransform.position - position;
		float distance = lightDir.length();
		
		if (distance > 0.001f) {
			lightDir = lightDir * (1.0f / distance);
		} else {
			continue;
		}
		
		// diffuse lighting (Lambertian)
		float diff = normal.dot(lightDir);
		diff = (diff > 0.0f) ? diff : 0.0f;
		
		// attenuation (how light falls off with distance)
		float attenuation = 1.0f / (light.constant + light.linear * distance + light.quadratic * distance * distance);
		
		// pply light contribution
		float lightFactor = diff * light.intensity * attenuation * fade;
		Vec3 diffuse = Vec3(
			baseColor.x * light.color.x * lightFactor,
			baseColor.y * light.color.y * lightFactor,
			baseColor.z * light.color.z * lightFactor
		);
		finalColor = finalColor + diffuse;
	}
	
	// clamp color values to [0, 1]
	finalColor.x = (finalColor.x > 1.0f) ? 1.0f : ((finalColor.x < 0.0f) ? 0.0f : finalColor.x);
	finalColor.y = (finalColor.y > 1.0f) ? 1.0f : ((finalColor.y < 0.0f) ? 0.0f : finalColor.y);
	finalColor.z = (finalColor.z > 1.0f) ? 1.0f : ((finalColor.z < 0.0f) ? 0.0f : finalColor.z);
	
	return finalColor;
}

float RenderSystem::computeLightRadius(const LightComponent& light) const {
	//TODO
	// solve the attenuation curve so we know when the light fades out so trhat we can get the radius of the light
	const float minContribution = 0.01f;
	const float target = light.intensity / minContribution;

	const float a = light.quadratic;
	const float b = light.linear;
	const float c = light.constant - target;

	if (std::abs(a) < 1e-4f) {
		if (std::abs(b) < 1e-4f) {
			return (light.constant > target) ? 0.0f : 0.0f;
		}
		float radius = -c / b;
		return radius > 0.0f ? radius : 0.0f;
	}

	float discriminant = b * b - 4.0f * a * c;
	if (discriminant < 0.0f) {
		return 0.0f;
	}

	float radius = (-b + std::sqrt(discriminant)) / (2.0f * a);
	return radius > 0.0f ? radius : 0.0f;
}

CameraComponent* RenderSystem::findActiveCamera() {
	// m_entityViews[1] is for cameras
	auto& cameraEntities = m_entityViews[1].dense();
	
	for (const Entity& entity : cameraEntities) {
		CameraComponent& camera = Engine::GetComponent<CameraComponent>(entity);
		if (camera.isActive) {
			return &camera;
		}
	}
	
	return nullptr;
}