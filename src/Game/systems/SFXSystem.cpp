#include "SFXSystem.h"

#include "app.h"
#include "FlowFieldSystem.h"
#include "engine/Engine.h"

void SFXSystem::Init() {
	Engine::GetSystem<FlowFieldSystem>().AddObserver(this);
}

void SFXSystem::Shutdown() {
	Engine::GetSystem<FlowFieldSystem>().RemoveObserver(this);
}


void SFXSystem::OnNotify(EventType type, const Event& payload) {

	if (type == EventType::FLOW_COMMAND_PLACED) App::PlayAudio("./data/TestData/placeNew.wav");
	if (type == EventType::FLOW_GRID_MOVED) App::PlayAudio("./data/TestData/move.wav");
}
