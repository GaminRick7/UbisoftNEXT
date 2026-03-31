///////////////////////////////////////////////////////////////////////////////
// Filename: GameTest.cpp
// This is the final version of the game!
///////////////////////////////////////////////////////////////////////////////

#if BUILD_PLATFORM_WINDOWS
//------------------------------------------------------------------------
#include <windows.h>
#endif

#include <iostream>
//------------------------------------------------------------------------
#include <math.h>
//------------------------------------------------------------------------
#include "../ContestAPI/app.h"
//------------------------------------------------------------------------
#include <engine/components/CameraComponent.h>

#include "AI/NavSystem.h"
#include "AI/AISystem.h"
#include "components/AIComponent.h"
#include "components/PlatformComponent.h"
#include "components/CrowdAgentComponent.h"
#include "components/CrowdGroupComponent.h"
#include "components/ArrowComponent.h"
#include "components/FlowCellComponent.h"
#include "engine/Engine.h"
#include "engine/graphics/Model.h"
#include "engine/components/MeshComponent.h"
#include "engine/graphics/RenderSystem.h"
#include "engine/graphics/CameraSystem.h"
#include "math/vec3.h"
#include "scenes/CrowdScene.h"
#include "scenes/MainMenuScene.h"
#include "systems/CrowdSimulationSystem.h"
#include "systems/FlowFieldSystem.h"
#include "systems/ArrowSystem.h"
#include "systems/SFXSystem.h"
#include "systems/TimeControlSystem.h"
//------------------------------------------------------------------------
// Example data....
//------------------------------------------------------------------------
Engine engine;
Entity m_camera;

//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Called before first update. Do any initial setup here.
//------------------------------------------------------------------------
void Init()
{
	Engine::Init(&engine);

	// Register components
	Engine::RegisterComponent<CameraComponent>();
	Engine::RegisterComponent<TransformComponent>();
	Engine::RegisterComponent<MeshComponent>();
	Engine::RegisterComponent<PlatformComponent>();
	Engine::RegisterComponent<AIComponent>();
	Engine::RegisterComponent<CrowdAgentComponent>();
	Engine::RegisterComponent<CrowdGroupComponent>();
	Engine::RegisterComponent<ArrowComponent>();
	Engine::RegisterComponent<FlowCellComponent>();

	// Register Systems
	Engine::RegisterSystem<NavSystem>();
	Engine::GetSystem<NavSystem>().Init();
	Engine::RegisterSystem<AISystem>();
	Engine::GetSystem<AISystem>().Init();
	Engine::RegisterSystem<FlowFieldSystem>();
	Engine::GetSystem<FlowFieldSystem>().Init();
	Engine::RegisterSystem<ArrowSystem>();
	Engine::GetSystem<ArrowSystem>().Init();
	Engine::RegisterSystem<CrowdSimulationSystem>();
	Engine::GetSystem<CrowdSimulationSystem>().Init();
	Engine::RegisterSystem<SFXSystem>();
	Engine::GetSystem<SFXSystem>().Init();
	Engine::RegisterSystem<TimeControlSystem>();
	Engine::GetSystem<TimeControlSystem>().Init();

	// Register Scenes
	Engine::RegisterScene(std::make_shared<CrowdScene>());
	Engine::RegisterScene(std::make_shared<MainMenuScene>());

	// play the main theme :D
	App::PlayAudio("./data/TestData/soundtrack.wav", true);

	// switch to main menu
	Engine::SwitchScene("MainMenu");
}

//------------------------------------------------------------------------
// Update your simulation here. deltaTime is the elapsed time since the last update in ms.
// This will be called at no greater frequency than the value of APP_MAX_FRAME_RATE
//------------------------------------------------------------------------
void Update(const float deltaTime)
{
	// Update engine systems and the current scene
	Engine::Update(deltaTime);
	Engine::UpdateActiveScene(deltaTime);
}

//------------------------------------------------------------------------
// Add your display calls here (DrawLine,Print, DrawSprite.)
// See App.h
//------------------------------------------------------------------------
void Render()
{
	Engine::Render();
}
//------------------------------------------------------------------------
// Add your shutdown code here. Called when the APP_QUIT_KEY is pressed.
// Just before the app exits.
//------------------------------------------------------------------------
void Shutdown() {
	App::StopAudio("./data/TestData/soundtrack.wav");
}