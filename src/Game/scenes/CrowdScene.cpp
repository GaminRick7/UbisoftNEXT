#include "CrowdScene.h"

#include "engine/Engine.h"
#include "AI/AISystem.h"
#include "AI/NavSystem.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/CameraComponent.h"
#include "engine/components/MeshComponent.h"
#include "engine/components/TransformComponent.h"
#include "engine/components/LightComponent.h"
#include "engine/components/UIPrintComponent.h"
#include "components/AIComponent.h"
#include "components/CrowdAgentComponent.h"
#include "components/CrowdGroupComponent.h"
#include "systems/CrowdSimulationSystem.h"
#include "systems/FlowFieldSystem.h"
#include "systems/ArrowSystem.h"
#include "math/MathUtils.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

#include "systems/TimeControlSystem.h"
#include "app.h"

void CrowdScene::Init() {
	Engine::GetSystem<CrowdSimulationSystem>().AddObserver(this);
	Engine::GetSystem<FlowFieldSystem>().Init();
	auto& flowSystem = Engine::GetSystem<FlowFieldSystem>();
	const int flowWidth = 10;
	const int flowLength = 10;
	const int goalX = 8;
	const int goalZ = 8;
	std::vector<int> heightMap = {
		0, 1, 1, 1, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 1, 3, 0, 3, 3, 0,
		0, 1, 0, 2, 2, 3, 0, 3, 3, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 2, 0, 0,
		0, 1, 2, 2, 0, 0, 0, 0, 0, 0,
		2, 1, 1, 1, 1, 1, 1, 0, 0, 0,
		2, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		4, 0, 3, 0, 0, 0, 1, 1, 2, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	};
	flowSystem.BuildFlowLevel(heightMap, flowWidth, flowLength, goalX, goalZ);

	// Floor plane
	Entity floor = Engine::CreateEntity();
	TransformComponent floorTransform;
	floorTransform.position = Vec3(0.0f, -2.0f, 0.0f);
	floorTransform.scale = Vec3(50.0f, 1.0f, 50.0f);
	floorTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
	Engine::AddComponent(floor, floorTransform);

	MeshComponent floorMesh;
	floorMesh.mesh = Engine::loadModel("./data/TestData/cube.obj");
	floorMesh.colour = Vec3(0.2f, 0.2f, 0.2f);
	Engine::AddComponent(floor, floorMesh);

	BoundsComponent floorBounds;
	floorBounds.bounds = floorMesh.mesh->bounds;
	Engine::AddComponent(floor, floorBounds);

	m_camera = Engine::CreateEntity();
	CameraComponent camera;
	camera.position = Vec3(0.0f, 60.0f, -80.0f);
	camera.target = Vec3(0.0f, 0.0f, 20.0f);
	camera.up = Vec3(0.0f, 1.0f, 0.0f);
	camera.mode = CameraComponent::Mode::Free;
	Engine::AddComponent(m_camera, camera);
	Engine::SetActiveCamera(m_camera);

	const float flowCellSize = flowSystem.GetCellSize();
	const Vec3& flowOrigin = flowSystem.GetFlowOrigin();
	const Vec3 flowCenter = Vec3(
		flowOrigin.x + flowCellSize * flowWidth * 0.5f,
		0.0f,
		flowOrigin.z + flowCellSize * flowLength * 0.5f
	);

	// flyby from start (0,0) to goal before gameplay starts.
	const Vec3 startCellCenter(
		flowOrigin.x + flowCellSize * 0.5f,
		0.0f,
		flowOrigin.z + flowCellSize * 0.5f
	);
	const Vec3 goalCellCenter(
		flowOrigin.x + flowCellSize * (static_cast<float>(goalX) + 0.5f),
		0.0f,
		flowOrigin.z + flowCellSize * (static_cast<float>(goalZ) + 0.5f)
	);
	m_goalCenter = goalCellCenter;
	m_flybyStart = startCellCenter + Vec3(0.0f, 35.0f, -flowCellSize * 2.0f);
	m_flybyEnd = goalCellCenter + Vec3(0.0f, 35.0f, flowCellSize * 2.0f);
	m_flybyStartLookAt = startCellCenter;
	m_flybyEndLookAt = goalCellCenter;
	m_flybyActive = true;
	m_flybyTimer = 0.0f;
	m_flybyForward = true;

	CameraComponent& flybyCamera = Engine::GetComponent<CameraComponent>(m_camera);
	flybyCamera.position = m_flybyStart;
	flybyCamera.target = m_flybyStartLookAt;
	flybyCamera.up = Vec3(0.0f, 1.0f, 0.0f);

	// orbit camera around the goal activated this camera on win
	m_goalOrbitCamera = Engine::CreateEntity();
	CameraComponent orbitCamera;
	orbitCamera.mode = CameraComponent::Mode::Orbital;
	orbitCamera.orbitCenter = m_goalCenter;
	orbitCamera.orbitCenterInitialized = true;
	orbitCamera.orbitRadius = 40.0f;
	orbitCamera.orbitHeight = 40.0f;
	orbitCamera.orbitSpeed = 0.18f;
	orbitCamera.orbitLookAtHeight = 15.0f;
	orbitCamera.fov = 110.0f;
	Engine::AddComponent(m_goalOrbitCamera, orbitCamera);

	auto& navSystem = Engine::GetSystem<NavSystem>();
	navSystem.PopulateFromTrackedEntities();

	std::vector<std::vector<Vec3>> allDronePoints = {
		{
			Vec3(5, 15, 55),
			Vec3(5, 15, 35),
			Vec3(25, 15, 35),
			Vec3(25, 15, 15),
			Vec3(35, 15, 15),
			Vec3(35, 15, 15),
			Vec3(35, 25, 15),
			Vec3(35, 25, 55),
			Vec3(35, 25, 55),
			Vec3(5, 25, 55),
			Vec3(5, 15, 55),
		},
		{
			Vec3(25, 5, 45),
			Vec3(45, 5, 45),
			Vec3(45, 15, 45),
			Vec3(45, 15, 75),
			Vec3(25, 15, 75),
			Vec3(25, 35, 75 ),
			Vec3(25, 35, 45),
			Vec3(25, 5, 45),
		},
		{
			Vec3(95, 5, 45),
			Vec3(85, 5, 45),
			Vec3(85, 25, 45),
			Vec3(65, 25, 45),
			Vec3(65, 5, 45),
			Vec3(65, 5, 5),
			Vec3(65, 5, 15),
			Vec3(65, 5, 15),
			Vec3(65, 35, 15),
			Vec3(85, 35, 15),
			Vec3(85, 35, 35),
			Vec3(85, 5, 35),
			Vec3(85, 5, 35),
			Vec3(85, 5, 45),
			Vec3(95, 5, 45),
		},
		{
			Vec3(15, 5, 95),
			Vec3(15, 5, 75),
			Vec3(35, 5, 75),
			Vec3(35, 5, 95),
			Vec3(15, 5, 95),
		},
	};

	std::vector<Vec3> lightColors = {
		Vec3(0.2f, 1.0f, 0.2f),
		Vec3(1.0f, 0.2f, 0.2f),
		Vec3(0.2f, 0.2f, 1.0f),
		Vec3(0.5f, 0.3f, 0.8f),
	};

	for (int i = 0; i < 4; ++i) {
		Entity drone = Engine::CreateEntity();
		TransformComponent droneTransform;
		droneTransform.position = allDronePoints[i][0];
		droneTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
		droneTransform.scale = Vec3(0.3f, 0.3f, 0.3f);
		Engine::AddComponent(drone, droneTransform);
	
		MeshComponent droneMesh;
		droneMesh.mesh = Engine::loadModel("./data/TestData/drone.obj");
		droneMesh.colour = Vec3(0.2f, 0.2f, 0.2f);
		Engine::AddComponent(drone, droneMesh);
	
		LightComponent droneLight;
		droneLight.color = lightColors[i];
		droneLight.intensity = 4.0f;
		droneLight.constant = 1.0f;
		droneLight.linear = 0.09f;
		droneLight.quadratic = 0.032f;
		Engine::AddComponent(drone, droneLight);
	
		BoundsComponent droneBounds;
		droneBounds.bounds = droneMesh.mesh->bounds;
		Engine::AddComponent(drone, droneBounds);
	
		AIComponent droneAI;
		droneAI.speed = 10.0f;
		droneAI.arriveDistance = 0.5f;
		droneAI.turnRate = 1000.0f;
	
		const std::vector<Vec3>& dronePoints = allDronePoints[i];
		droneAI.path = dronePoints;
		// std::vector<Vec3> fullPath;
		// for (size_t i = 0; i < dronePoints.size(); ++i) {
		// 	const Vec3& start = dronePoints[i];
		// 	const Vec3& end = dronePoints[(i + 1) % dronePoints.size()];
		// 	const std::vector<Vec3>& segmentPath = navSystem.GetPath(start, end);
		// 	fullPath.insert(fullPath.end(), segmentPath.begin(), segmentPath.end());
		// }
		// droneAI.path = navSystem.BuildAxisAlignedPath(fullPath, heightMap, flowWidth, flowLength, flowOrigin, flowCellSize, 5.0f);
		Engine::AddComponent(drone, droneAI);
		if (i == 0) {
			m_drone1Entity = drone;
		}
	}


	//TODO Remove
	auto createPointLight = [](const Vec3& position, const Vec3& color, float intensity) {
		Entity light = Engine::CreateEntity();
		TransformComponent lightTransform;
		lightTransform.position = position;
		lightTransform.scale = Vec3(1.0f, 1.0f, 1.0f);
		lightTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
		Engine::AddComponent(light, lightTransform);

		LightComponent lightComp;
		lightComp.color = color;
		lightComp.intensity = intensity;
		lightComp.constant = 1.0f;
		lightComp.linear = 0.09f;
		lightComp.quadratic = 0.032f;
		Engine::AddComponent(light, lightComp);
	};

	const float lightHeight = 10.0f;
	const float margin = flowCellSize * 1.5f;
	const float minX = flowOrigin.x - margin;
	const float maxX = flowOrigin.x + flowCellSize * static_cast<float>(flowWidth) + margin;
	const float minZ = flowOrigin.z - margin;
	const float maxZ = flowOrigin.z + flowCellSize * static_cast<float>(flowLength) + margin;

	// 10 lights, weighted toward corners/edges.
	createPointLight(Vec3(minX, lightHeight, minZ), Vec3(1.0f, 0.9f, 0.8f), 9.0f);
	createPointLight(Vec3(maxX, lightHeight, minZ), Vec3(0.9f, 0.95f, 1.0f), 9.0f);
	createPointLight(Vec3(minX, lightHeight, maxZ), Vec3(0.9f, 1.0f, 0.85f), 9.0f);
	createPointLight(Vec3(maxX, lightHeight, maxZ), Vec3(1.0f, 0.9f, 1.0f), 9.0f);

	const float midX = (minX + maxX) * 0.5f;
	const float midZ = (minZ + maxZ) * 0.5f;
	createPointLight(Vec3(midX, lightHeight, minZ), Vec3(0.9f, 0.85f, 1.0f), 7.0f);
	createPointLight(Vec3(midX, lightHeight, maxZ), Vec3(0.85f, 1.0f, 0.9f), 7.0f);
	createPointLight(Vec3(minX, lightHeight, midZ), Vec3(1.0f, 0.9f, 0.85f), 7.0f);
	createPointLight(Vec3(maxX, lightHeight, midZ), Vec3(0.85f, 0.9f, 1.0f), 7.0f);

	createPointLight(flowCenter + Vec3(0.0f, lightHeight, -flowCellSize * 2.5f), Vec3(1.0f, 0.95f, 0.9f), 6.0f);
	createPointLight(flowCenter + Vec3(0.0f, lightHeight, flowCellSize * 2.5f), Vec3(0.9f, 1.0f, 0.95f), 6.0f);

	m_rng = std::mt19937(std::random_device{}());
}

void CrowdScene::OnNotify(EventType type, const Event& /*payload*/) {
	if (type != EventType::GOAL_COUNT_REACHED || m_goalTextShown) {
		return;
	}
	m_goalTextEntity = Engine::CreateEntity();
	UIPrintComponent goalText;
	goalText.x = APP_VIRTUAL_WIDTH * 0.5f - 32.0f;
	goalText.y = APP_VIRTUAL_HEIGHT * 0.5f;
	goalText.text = "You win.";
	goalText.r = 1.0f;
	goalText.g = 1.0f;
	goalText.b = 1.0f;
	goalText.font = GLUT_BITMAP_HELVETICA_18;
	Engine::AddComponent(m_goalTextEntity, goalText);
	m_goalTextShown = true;

	m_returnTextEntity = Engine::CreateEntity();
	UIPrintComponent returnText;
	returnText.x = APP_VIRTUAL_WIDTH * 0.5f - 100.0f;
	returnText.y = APP_VIRTUAL_HEIGHT * 0.5f - 30.0f;
	returnText.text = "Press any button to return to main menu.";
	returnText.r = 1.0f;
	returnText.g = 1.0f;
	returnText.b = 1.0f;
	returnText.font = GLUT_BITMAP_HELVETICA_12;
	Engine::AddComponent(m_returnTextEntity, returnText);
	m_waitingForReturn = true;

	m_flybyActive = false;
	Engine::SetActiveCamera(m_goalOrbitCamera);
}

void CrowdScene::Update(float deltaTime) {
	const float deltaSeconds = deltaTime * 0.001f;
	if (deltaSeconds <= 0.0f) {
		return;
	}

	if (m_waitingForReturn) {
		const CController& controller = App::GetController();
		const bool anyButtonPressed =
			controller.CheckButton(App::BTN_A) ||
			controller.CheckButton(App::BTN_B) ||
			controller.CheckButton(App::BTN_X) ||
			controller.CheckButton(App::BTN_Y) ||
			controller.CheckButton(App::BTN_START) ||
			controller.CheckButton(App::BTN_BACK) ||
			controller.CheckButton(App::BTN_LBUMPER) ||
			controller.CheckButton(App::BTN_RBUMPER) ||
			controller.CheckButton(App::BTN_DPAD_UP) ||
			controller.CheckButton(App::BTN_DPAD_DOWN) ||
			controller.CheckButton(App::BTN_DPAD_LEFT) ||
			controller.CheckButton(App::BTN_DPAD_RIGHT) ||
			App::IsKeyPressed(App::KEY_SPACE) ||
			App::IsKeyPressed(App::KEY_ENTER) ||
			App::IsKeyPressed(App::KEY_ESC);
		if (anyButtonPressed) {
			Engine::SwitchScene("MainMenu");
			return;
		}
	}

	if (m_flybyActive) {
		m_flybyTimer += deltaSeconds;
		const float tRaw = m_flybyTimer / m_flybyDuration;
		const float t = (std::min)((std::max)(tRaw, 0.0f), 1.0f);
		const float smoothT = t * t * (3.0f - 2.0f * t);

		const Vec3 flybyStart = m_flybyForward ? m_flybyStart : m_flybyEnd;
		const Vec3 flybyEnd = m_flybyForward ? m_flybyEnd : m_flybyStart;
		const Vec3 flybyStartLookAt = m_flybyForward ? m_flybyStartLookAt : m_flybyEndLookAt;
		const Vec3 flybyEndLookAt = m_flybyForward ? m_flybyEndLookAt : m_flybyStartLookAt;

		CameraComponent& camera = Engine::GetComponent<CameraComponent>(m_camera);
		camera.position = lerp(flybyStart, flybyEnd, smoothT);
		camera.target = lerp(flybyStartLookAt, flybyEndLookAt, smoothT);
		camera.up = Vec3(0.0f, 1.0f, 0.0f);

		if (t >= 1.0f) {
			if (m_flybyForward) {
				m_flybyTimer = 0.0f;
				m_flybyForward = false;
			} else {
				m_flybyActive = false;
			}
		}
		return;
	}

	m_spawnAccumulator += deltaSeconds;
	while (m_spawnAccumulator >= m_spawnInterval) {
		m_spawnAccumulator -= m_spawnInterval;

		auto& crowdSystem = Engine::GetSystem<CrowdSimulationSystem>();
		const auto& flowSystem = Engine::GetSystem<FlowFieldSystem>();
		const float flowCellSize = flowSystem.GetCellSize();
		const Vec3& flowOrigin = flowSystem.GetFlowOrigin();
		if (m_currentGroupId < 0 || m_spawnedCount >= m_agentsPerGroup) {
			const int newGroupId = crowdSystem.AcquireGroupId();
			Entity groupEntity = Engine::CreateEntity();

			TransformComponent groupTransform;
			groupTransform.position = Vec3(
				flowOrigin.x + flowCellSize * 0.5f,
				0.0f,
				flowOrigin.z + flowCellSize * 0.5f
			);
			groupTransform.scale = Vec3(1.0f, 1.0f, 1.0f);
			groupTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
			Engine::AddComponent(groupEntity, groupTransform);

			BoundsComponent groupBounds;
			groupBounds.bounds = AABB(
				Vec3(-0.25f, -0.25f, -0.25f),
				Vec3(0.25f, 0.25f, 0.25f)
			);
			Engine::AddComponent(groupEntity, groupBounds);

			CrowdGroupComponent groupComp;
			groupComp.groupId = newGroupId;
			Engine::AddComponent(groupEntity, groupComp);
			m_spawnedCount = 0;
			m_currentGroupId = newGroupId;

			crowdSystem.MarkGroupsDirty();
		}

		Entity agent = Engine::CreateEntity();
		TransformComponent transform;
		const float agentHeight = m_heightDist(m_rng);
		const float agentWidth = m_widthDist(m_rng);
		transform.scale = Vec3(agentWidth, agentHeight, agentWidth);
		const float spawnHeightOffset = transform.scale.y + agentHeight/2;
		transform.position = Vec3(
			flowOrigin.x + flowCellSize * 0.5f + m_spawnJitter(m_rng),
			spawnHeightOffset,
			flowOrigin.z + flowCellSize * 0.5f + m_spawnJitter(m_rng)
		);
		transform.rotation = Vec3(0.0f, 0.0f, 0.0f);
		Engine::AddComponent(agent, transform);

		MeshComponent mesh;
		mesh.mesh = Engine::loadModel("./data/TestData/cube.obj");
		mesh.colour = Vec3(m_colorDist(m_rng), m_colorDist(m_rng), m_colorDist(m_rng));
		Engine::AddComponent(agent, mesh);

		// ColliderComponent collider;
		// collider.type = ColliderComponent::Type::AABB;
		// collider.box = mesh.mesh->bounds;
		// Engine::AddComponent(agent, collider);

		CrowdAgentComponent crowdAgent;
		crowdAgent.position = transform.position;
		crowdAgent.velocity = Vec3(0.0f, 0.0f, 0.0f);
		crowdAgent.groupId = m_currentGroupId;
		Engine::AddComponent(agent, crowdAgent);
		crowdSystem.IncrementGroupCount(m_currentGroupId);
		m_spawnedCount += 1;
	}

	Engine::GetSystem<FlowFieldSystem>().Update(deltaTime);
	Engine::GetSystem<ArrowSystem>().Update(deltaTime);
	Engine::GetSystem<AISystem>().Update(deltaTime);
	Engine::GetSystem<TimeControlSystem>().Update(deltaTime);
	Engine::GetSystem<CrowdSimulationSystem>().Update(deltaTime);
}


void CrowdScene::Shutdown() {
	Engine::GetSystem<CrowdSimulationSystem>().RemoveObserver(this);
}
