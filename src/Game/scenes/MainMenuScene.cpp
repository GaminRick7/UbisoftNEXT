#include "MainMenuScene.h"

#include <vector>

#include "engine/Engine.h"
#include "app.h"
#include "engine/components/CameraComponent.h"
#include "engine/components/TransformComponent.h"
#include "engine/components/MeshComponent.h"
#include "engine/components/BoundsComponent.h"
#include "engine/components/LightComponent.h"
#include "engine/components/UIPrintComponent.h"
#include "components/CrowdAgentComponent.h"
#include "components/CrowdGroupComponent.h"
#include "systems/CrowdSimulationSystem.h"
#include "systems/ArrowSystem.h"
#include "systems/FlowFieldSystem.h"

void MainMenuScene::Init() {

	//initiazlize flow field
	auto& flowSystem = Engine::GetSystem<FlowFieldSystem>();
	flowSystem.Init();

	Engine::GetSystem<CrowdSimulationSystem>().SetMaxSpeed(12.0f);

	// GRID SETUP
	const int gridWidth = 5;
	const int gridHeight = 5;
	std::vector<int> heightMap = {
		1, 1, 0, 1, 1,
		1, 0, 0, 0, 1,
		0, 0, 2, 0, 0,
		1, 0, 0, 0, 1,
		1, 1, 0, 1, 1,
	};
	flowSystem.BuildFlowLevel(heightMap, gridWidth, gridHeight, 2, 2);

	// pre-place a few flow commands
	flowSystem.PlaceCommand(0, 0, Vec3(1.0f, 0.0f, 0.0f));
	flowSystem.PlaceCommand(1, 0, Vec3(0.0f, 1.0f, 0.0f));
	flowSystem.PlaceCommand(4, 0, Vec3(0.0f, 0.0f, 1.0f));
	flowSystem.PlaceCommand(4, 1, Vec3(0.0f, 1.0f, 0.0f));
	flowSystem.PlaceCommand(4, 4, Vec3(-1.0f, 0.0f, 0.0f));
	flowSystem.PlaceCommand(3, 4, Vec3(0.0f, 1.0f, 0.0f));
	flowSystem.PlaceCommand(0, 4, Vec3(0.0f, 0.0f, -1.0f));
	flowSystem.PlaceCommand(0, 3, Vec3(0.0f, 1.0f, 0.0f));
	
	// save the center or calculations ahead
	m_flowCellSize = flowSystem.GetCellSize();
	m_flowOrigin = flowSystem.GetFlowOrigin();
	const Vec3 center = Vec3(
		m_flowOrigin.x + m_flowCellSize * gridWidth * 0.5f,
		0.0f,
		m_flowOrigin.z + m_flowCellSize * gridHeight * 0.5f
	);

	// spawn floor and associated components
	Entity floor = Engine::CreateEntity();
	TransformComponent floorTransform;
	floorTransform.position = Vec3(center.x, -0.5f, center.z);
	floorTransform.scale = Vec3(1.0f, 1.0f, 1.0f);
	floorTransform.rotation = Vec3(0.0f, 0.0f, 0.0f);
	Engine::AddComponent(floor, floorTransform);

	MeshComponent floorMesh;
	floorMesh.mesh = Engine::loadModel("./data/TestData/cube.obj");
	floorMesh.colour = Vec3(0.2f, 0.2f, 0.2f);
	Engine::AddComponent(floor, floorMesh);

	BoundsComponent floorBounds;
	floorBounds.bounds = floorMesh.mesh->bounds;
	Engine::AddComponent(floor, floorBounds);

	// start up a rng to be used at different places

	m_rng = std::mt19937(std::random_device{}());

	// UI Title
	Entity title = Engine::CreateEntity();
	UIPrintComponent titleText;
	titleText.x = APP_VIRTUAL_WIDTH * 0.5f - 48.0f;
	titleText.y = APP_VIRTUAL_HEIGHT * 0.5f;
	titleText.text = "BAGGAGE";
	titleText.r = 1.0f;
	titleText.g = 1.0f;
	titleText.b = 1.0f;
	titleText.font = GLUT_BITMAP_HELVETICA_18;
	Engine::AddComponent(title, titleText);

	// UI Subtitle
	m_promptTextEntity = Engine::CreateEntity();
	UIPrintComponent promptText;
	promptText.x = APP_VIRTUAL_WIDTH * 0.5f - 80.0f;
	promptText.y = 30.0f;
	promptText.text = "[ Press Any Button To Start ]";
	promptText.r = 1.0f;
	promptText.g = 1.0f;
	promptText.b = 1.0f;
	promptText.font = GLUT_BITMAP_HELVETICA_12;
	Engine::AddComponent(m_promptTextEntity, promptText);

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

	// key light up points
	const float lightHeight = m_flowOrigin.y + 6.0f;
	const float margin = m_flowCellSize * 0.75f;
	const float minX = m_flowOrigin.x - margin;
	const float maxX = m_flowOrigin.x + m_flowCellSize * gridWidth + margin;
	const float minZ = m_flowOrigin.z - margin;
	const float maxZ = m_flowOrigin.z + m_flowCellSize * gridHeight + margin;
	const float midX = (minX + maxX) * 0.5f;
	const float midZ = (minZ + maxZ) * 0.5f;

	createPointLight(Vec3(minX, lightHeight, minZ), Vec3(1.0f, 0.9f, 0.8f), 6.0f);
	createPointLight(Vec3(maxX, lightHeight, minZ), Vec3(0.9f, 0.95f, 1.0f), 6.0f);
	createPointLight(Vec3(minX, lightHeight, maxZ), Vec3(0.9f, 1.0f, 0.85f), 6.0f);
	createPointLight(Vec3(maxX, lightHeight, maxZ), Vec3(1.0f, 0.9f, 1.0f), 6.0f);
	createPointLight(Vec3(midX, lightHeight, minZ), Vec3(0.9f, 0.85f, 1.0f), 5.0f);
	createPointLight(Vec3(midX, lightHeight, maxZ), Vec3(0.85f, 1.0f, 0.9f), 5.0f);
	createPointLight(Vec3(minX, lightHeight, midZ), Vec3(1.0f, 0.9f, 0.85f), 5.0f);
	createPointLight(Vec3(maxX, lightHeight, midZ), Vec3(0.85f, 0.9f, 1.0f), 5.0f);

	//create the orbital camera and point it at the center
	m_orbitCamera = Engine::CreateEntity();
	CameraComponent orbitCamera;
	orbitCamera.position = Vec3(center.x, 12.0f, center.z - 12.0f);
	orbitCamera.target = center;
	orbitCamera.up = Vec3(0.0f, 1.0f, 0.0f);
	orbitCamera.mode = CameraComponent::Mode::Orbital;
	orbitCamera.orbitRadius = 45.0f;
	orbitCamera.orbitHeight = 18.0f;
	orbitCamera.orbitSpeed = 0.07f;
	orbitCamera.orbitLookAtHeight = 12.0f;
	orbitCamera.orbitCenter = center;
	orbitCamera.orbitCenterInitialized = true;
	orbitCamera.fov = 120.0f;
	Engine::AddComponent(m_orbitCamera, orbitCamera);

	//create the top down camera and point it at the center
	m_topDownCamera = Engine::CreateEntity();
	CameraComponent topDownCamera;
	topDownCamera.position = Vec3(center.x, center.y + 40.0f, center.z);
	topDownCamera.target = center;
	topDownCamera.mode = CameraComponent::Mode::TopDown;
	topDownCamera.topDownHeight = 40.0f;
	topDownCamera.topDownSmooth = 6.0f;
	topDownCamera.topDownYawSpeed = -0.1f;
	topDownCamera.fov = 130.0f;
	Engine::AddComponent(m_topDownCamera, topDownCamera);

	Engine::SetActiveCamera(m_orbitCamera);
}

void MainMenuScene::Update(float deltaTime) {
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
		Engine::SwitchScene("CrowdScene");
		return;
	}

	// lets lkeep track of  the time elapsed and switch between the two cameras for a cool effect
	m_cameraSwitchTimer += deltaTime * 0.001f;
	if (m_cameraSwitchTimer >= 10.0f) {
		m_cameraSwitchTimer = 0.0f;
		m_useOrbitCamera = !m_useOrbitCamera;
		Engine::SetActiveCamera(m_useOrbitCamera ? m_orbitCamera : m_topDownCamera);
	}

	// replace the text with blank text for the subtitle periodically to create a flashing efffect
	m_promptFlashTimer += deltaTime * 0.001f;
	if (m_promptFlashTimer >= m_promptFlashInterval) {
		m_promptFlashTimer = 0.0f;
		m_promptVisible = !m_promptVisible;
		if (Engine::HasComponent<UIPrintComponent>(m_promptTextEntity)) {
			UIPrintComponent& promptText = Engine::GetComponent<UIPrintComponent>(m_promptTextEntity);
			promptText.text = m_promptVisible ? "[Press Any Button To Start]" : "";
		}
	}

	// ENtity spwaning logic - same as the main scene except we have a mqax entity count
	const float deltaSeconds = deltaTime * 0.001f;
	if (deltaSeconds > 0.0f) {
		m_spawnAccumulator += deltaSeconds;
		while (m_spawnAccumulator >= m_spawnInterval) {
			m_spawnAccumulator -= m_spawnInterval;

			// if the spawned enity count surpasses 500, we stop spawning
			if (Engine::GetEntityCount() >= 500) {
				m_spawnAccumulator = 0.0f;
				break;
			}

			auto& crowdSystem = Engine::GetSystem<CrowdSimulationSystem>();
			if (m_currentGroupId < 0 || m_spawnedCount >= m_agentsPerGroup) {
				const int newGroupId = crowdSystem.AcquireGroupId();
				Entity groupEntity = Engine::CreateEntity();

				TransformComponent groupTransform;
				groupTransform.position = Vec3(
					m_flowOrigin.x + m_flowCellSize * 0.5f,
					0.0f,
					m_flowOrigin.z + m_flowCellSize * 0.5f
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

				crowdSystem.MarkGroupsDirty();
				m_spawnedCount = 0;
				m_currentGroupId = newGroupId;
			}

			Entity agent = Engine::CreateEntity();
			TransformComponent transform;
			const float agentHeight = m_heightDist(m_rng);
			const float agentWidth = m_widthDist(m_rng);
			transform.scale = Vec3(agentWidth, agentHeight, agentWidth);
			const float spawnHeightOffset = transform.scale.y + agentHeight / 2.0f;
			transform.position = Vec3(
				m_flowOrigin.x + m_flowCellSize * 0.5f + m_spawnJitter(m_rng),
				spawnHeightOffset,
				m_flowOrigin.z + m_flowCellSize * 0.5f + m_spawnJitter(m_rng)
			);
			transform.rotation = Vec3(0.0f, 0.0f, 0.0f);
			Engine::AddComponent(agent, transform);

			MeshComponent mesh;
			mesh.mesh = Engine::loadModel("./data/TestData/cube.obj");
			mesh.colour = Vec3(m_colorDist(m_rng), m_colorDist(m_rng), m_colorDist(m_rng));
			Engine::AddComponent(agent, mesh);

			CrowdAgentComponent crowdAgent;
			crowdAgent.position = transform.position;
			crowdAgent.velocity = Vec3(0.0f, 0.0f, 0.0f);
			crowdAgent.groupId = m_currentGroupId;
			Engine::AddComponent(agent, crowdAgent);

			crowdSystem.IncrementGroupCount(m_currentGroupId);
			m_spawnedCount += 1;
		}
	}

	Engine::GetSystem<FlowFieldSystem>().Update(deltaTime);
	Engine::GetSystem<ArrowSystem>().Update(deltaTime);
	Engine::GetSystem<CrowdSimulationSystem>().Update(deltaTime);
}

void MainMenuScene::Shutdown() {
}
